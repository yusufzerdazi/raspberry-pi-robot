import socket
import threading
import time
import math

from BrickPi import *
from config import SENSING
import brickpi

# Speeds
DEFAULT_MOVEMENT_SPEED = 255  # Default movement speed
DEFAULT_ROTATION_SPEED = 200  # Default rotation speed

# Constants
ROBOT_WIDTH = 14  # Distance between wheels (cm)
CM_PER_DEGREE = 36.1/720.0  # How far the wheels move for each degree of encoder rotation
TURN_DISTANCE = (math.pi*14)/4


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
            brickpi.MotorRotateDegree([DEFAULT_ROTATION_SPEED], [90], [PORT_D], False)
            brickpi.MotorRotateDegree([DEFAULT_ROTATION_SPEED], [-90], [PORT_D], False)
            brickpi.MotorRotateDegree([DEFAULT_ROTATION_SPEED], [0], [PORT_D], False)

    def resume(self):
        with self.state:
            self.paused = False
            self.state.notify()  # unblock self if waiting

    def pause(self):
        with self.state:
            self.paused = True  # make self block and wait

    def stop(self):
        self.running = False


class Communication(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.udp_ip = "192.168.0.7"  # UDP IP Address
        self.udp_port = 5005  # UDP Port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.udp_port))
        self.running = True

    def sense(self):
        right_wheel = BrickPi.Encoder[PORT_B]
        left_wheel = BrickPi.Encoder[PORT_C]
        angle = BrickPi.Encoder[PORT_D]
        forward = BrickPi.Sensor[PORT_2]
        backward = BrickPi.Sensor[PORT_3]
        return [right_wheel, left_wheel, angle, forward, backward]

    def run(self):
        while self.running:
            BrickPiUpdateValues()
            status = self.sense()
            self.sock.sendto(",".join([str(val) for val in status]), (self.udp_ip, self.udp_port))
            time.sleep(.05)
        self.sock.close()

    def stop(self):
        self.running = False
        

class Robot(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.scan = Scan()
        self.communication = Communication()

    def forward(self, speed=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[PORT_B] = speed
        BrickPi.MotorSpeed[PORT_C] = speed

    def backward(self, speed=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[PORT_B] = -speed
        BrickPi.MotorSpeed[PORT_C] = -speed

    def turn(self, angle):
        arc_length = math.pi*14*(float(angle)/360.0)
        angle = int(arc_length/CM_PER_DEGREE)
        brickpi.MotorRotateDegree([DEFAULT_MOVEMENT_SPEED, DEFAULT_MOVEMENT_SPEED], [angle, -angle], [PORT_B, PORT_C])

    def right(self, speed=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[PORT_B] = -speed
        BrickPi.MotorSpeed[PORT_C] = speed

    def left(self, speed=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[PORT_B] = speed
        BrickPi.MotorSpeed[PORT_C] = -speed

    def still(self, speed=DEFAULT_MOVEMENT_SPEED):
        BrickPi.MotorSpeed[PORT_B] = 0
        BrickPi.MotorSpeed[PORT_C] = 0

    def run(self):
        self.communication.start()
        self.scan.start()
        while self.running:
            command, address = self.communication.sock.recvfrom(1024)
            self.still()
            if command == "W":
                self.forward()
            elif command == "S":
                self.backward()
            elif command == "D":
                self.right()#turn(90)
            elif command == "A":
                self.left()#turn(-90)
            elif command == "Z":
                self.still()
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
    BrickPi.MotorEnable[PORT_B] = 1
    BrickPi.MotorEnable[PORT_C] = 1
    BrickPi.MotorEnable[PORT_D] = 1

    # Enable the ultrasonic sensors
    BrickPi.SensorType[PORT_2] = TYPE_SENSOR_ULTRASONIC_CONT
    BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT

    # Setup the sensors
    BrickPiSetupSensors()
    BrickPiUpdateValues()

    # Set the encoder offset to the encoder value (so they are initialised at 0)
    BrickPi.EncoderOffset[PORT_B] = BrickPi.Encoder[PORT_B]
    BrickPi.EncoderOffset[PORT_C] = BrickPi.Encoder[PORT_C]
    BrickPi.EncoderOffset[PORT_D] = BrickPi.Encoder[PORT_D]
    BrickPiUpdateValues()

    # Start the robot thread
    robot = Robot()
    robot.start()
