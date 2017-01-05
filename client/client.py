#!/usr/bin/env python
# Yusuf Zerdazi

from BrickPi import *
import socket
import threading
import time
import math

from config import SENSING 

UDP_IP = "192.168.0.17"  # UDP IP Address
UDP_PORT = 5005  # UDP Port
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
SOCK.bind(("", UDP_PORT))

BrickPiSetup()  # setup the serial port for communication

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

# Speeds
DEFAULT_MOVEMENT_SPEED = 255  # Default movement speed
DEFAULT_ROTATION_SPEED = 150  # Default rotation speed

# Constants
ROBOT_WIDTH = 14 # Distance between wheels (cm)
CM_PER_DEGREE = 36.1/720.0
TURN_DISTANCE = (math.pi*14)/4

RUNNING = True


def run():
    global RUNNING
    while RUNNING:
        robot_state = sense()
        SOCK.sendto(",".join([str(val) for val in robot_state]), (UDP_IP, UDP_PORT))
        #BrickPiUpdateValues()       # Ask BrickPi to update values for senso
        time.sleep(.15)              # sleep for 100 ms
    return 0


def stop_sensor():
    global SENSING
    SENSING = False


def start_sensor():
    global SENSING
    SENSING = True


def sense():
    right_wheel = BrickPi.Encoder[PORT_B]
    left_wheel = BrickPi.Encoder[PORT_C]
    angle = BrickPi.Encoder[PORT_D]
    forward = BrickPi.Sensor[PORT_2]
    backward = BrickPi.Sensor[PORT_3]
    return [right_wheel, left_wheel, angle, forward, backward]


def forward(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = speed
    BrickPi.MotorSpeed[PORT_C] = speed


def backward(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = -speed
    BrickPi.MotorSpeed[PORT_C] = -speed


def turn(angle):
    arc_length = math.pi*14*(float(angle)/360.0)
    angle = int(arc_length/CM_PER_DEGREE)
    print angle
    motorRotateDegree([DEFAULT_MOVEMENT_SPEED, DEFAULT_MOVEMENT_SPEED], [angle, -angle], [PORT_B, PORT_C])


def right(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = speed
    BrickPi.MotorSpeed[PORT_C] = -speed


def left(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = -speed
    BrickPi.MotorSpeed[PORT_C] = speed


def stop(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = 0
    BrickPi.MotorSpeed[PORT_C] = 0


def scan():
    global RUNNING, SENSING
    while RUNNING:
        if SENSING:
            motorRotateDegree([DEFAULT_ROTATION_SPEED], [90], [PORT_D], False)
            motorRotateDegree([DEFAULT_ROTATION_SPEED], [-90], [PORT_D], False)
            motorRotateDegree([DEFAULT_ROTATION_SPEED], [0], [PORT_D], False)
        else:
            BrickPi.MotorSpeed[PORT_D] = 0
    return 0


def move():
    global RUNNING
    while RUNNING:
        command, address = SOCK.recvfrom(1024)

        if command == "W":
            forward()
        elif command == "S":
            backward()
        elif command == "D":
            right()
        elif command == "A":
            left()
        elif command == "Z":
            stop()
        elif command == "X":
            stop_sensor()
        elif command == "C":
            start_sensor()
        elif command == "STOP":
            stop()
            RUNNING = False
            SENSING = False
            SOCK.close()
    return 0


if __name__ == "__main__":
    threading.Thread(target=run).start()
    threading.Thread(target=scan).start()
    threading.Thread(target=move).start()
