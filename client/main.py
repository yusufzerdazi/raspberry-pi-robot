import socket
import threading
import time
import math

from BrickPi import *
from config import SENSING
import brickpi

# Socket
UDP_IP = "10.245.131.89"  # UDP IP Address
UDP_PORT = 5005  # UDP Port
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
SOCK.bind(("", UDP_PORT))

# Speeds
DEFAULT_MOVEMENT_SPEED = 255  # Default movement speed
DEFAULT_ROTATION_SPEED = 200  # Default rotation speed

# Port variables
RIGHT_WHEEL = PORT_B
LEFT_WHEEL = PORT_C
SENSOR = PORT_D
FRONT_SENSOR = PORT_2
REAR_SENSOR = PORT_3


class Scan(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.paused = True  # start out paused
        self.running = True
        self.state = threading.Condition()

    def run(self):
        self.resume() # unpause self
        while self.running:
            with self.state:
                if self.paused:
                    self.state.wait() # block until notified
            self.rotate(DEFAULT_ROTATION_SPEED, 90, False)
            self.rotate(DEFAULT_ROTATION_SPEED, -90, False)

    def rotate(self, power, deg, sampling_time=0.05):
        finished = False
        current = BrickPi.Encoder[SENSOR]
        final = deg*2
        rotate = (final > current) - (final < current)
        BrickPi.MotorSpeed[SENSOR] = rotate*power
        
        while (not finished) and (not self.paused) and (self.running):
            if (rotate > 0 and final > current) or (rotate < 0 and final < current):
                current = BrickPi.Encoder[SENSOR]
            else:
                finished = True
            time.sleep(sampling_time)
        BrickPi.MotorSpeed[SENSOR] = 0
        return 0
        
    def resume(self):
        with self.state:
            self.paused = False
            self.state.notify()  # unblock self if waiting

    def pause(self):
        with self.state:
            self.paused = True  # make self block and wait

    def stop(self):
        with self.state:
            self.running = False
            self.state.notify()  # unblock self if waiting


class Move(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.state = threading.Condition()

    def turn(self, direction=1, power=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[RIGHT_WHEEL] = direction*power
        BrickPi.MotorSpeed[LEFT_WHEEL] = -direction*power

    def drive(self, direction=1, power=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[RIGHT_WHEEL] = direction*power
        BrickPi.MotorSpeed[LEFT_WHEEL] = direction*power

    def stop(self):
        BrickPi.MotorSpeed[LEFT_WHEEL] = 0
        BrickPi.MotorSpeed[RIGHT_WHEEL] = 0


class Communication(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True

    def sense(self):
        right_wheel = BrickPi.Encoder[RIGHT_WHEEL]
        left_wheel = BrickPi.Encoder[LEFT_WHEEL]
        angle = BrickPi.Encoder[SENSOR]
        forward = BrickPi.Sensor[FRONT_SENSOR]
        backward = BrickPi.Sensor[REAR_SENSOR]
        return [right_wheel, left_wheel, angle, forward, backward]

    def run(self):
        while self.running:
            BrickPiUpdateValues()
            status = self.sense()
            SOCK.sendto(",".join([str(val) for val in status]), (UDP_IP, UDP_PORT))
            time.sleep(.05)
        SOCK.close()

    def stop(self):
        self.running = False
        

class Robot(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.scan = Scan()
        self.communication = Communication()
        self.movement = Move()

    def run(self):
        self.communication.start()
        self.scan.start()
        self.movement.start()
        while self.running:
            command, address = SOCK.recvfrom(1024)
            if command == "W":
                self.movement.drive()
            elif command == "S":
                self.movement.drive(-1)
            elif command == "D":
                self.movement.turn(-1)
            elif command == "A":
                self.movement.turn()
            elif command == "Z":
                self.movement.stop()
            elif command == "X":
                self.scan.pause()
            elif command == "C":
                self.scan.resume()
            elif command == "STOP":
                self.stop()
        self.scan.stop()
        self.scan.join()
        self.communication.stop()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    # Perform BrickPi initialisation
    BrickPiSetup()

    # Enable the motors
    BrickPi.MotorEnable[LEFT_WHEEL] = 1
    BrickPi.MotorEnable[RIGHT_WHEEL] = 1
    BrickPi.MotorEnable[SENSOR] = 1

    # Enable the ultrasonic sensors
    BrickPi.SensorType[FRONT_SENSOR] = TYPE_SENSOR_ULTRASONIC_CONT
    BrickPi.SensorType[REAR_SENSOR] = TYPE_SENSOR_ULTRASONIC_CONT

    # Setup the sensors
    BrickPiSetupSensors()
    BrickPiUpdateValues()

    # Set the encoder offset to the encoder value (so they are initialised at 0)
    BrickPi.EncoderOffset[LEFT_WHEEL] = BrickPi.Encoder[LEFT_WHEEL]
    BrickPi.EncoderOffset[RIGHT_WHEEL] = BrickPi.Encoder[RIGHT_WHEEL]
    BrickPi.EncoderOffset[SENSOR] = BrickPi.Encoder[SENSOR]
    BrickPiUpdateValues()

    # Start the robot thread
    robot = Robot()
    robot.start()
