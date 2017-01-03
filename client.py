#!/usr/bin/env python
# Yusuf Zerdazi

from BrickPi import *
import socket
import threading
import time

UDP_IP = "192.168.0.17"  # UDP IP Address
UDP_PORT = 5005  # UDP Port
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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
DEFAULT_ROTATION_SPEED = 100  # Default rotation speed

# Sensor values
RIGHT_WHEEL = 0
LEFT_WHEEL = 0
ANGLE = 0
FORWARD = 0
BACKWARD = 0

RUNNING = True


def rotate_sensor(deg, sampling_time=.1, delay_when_stopping=.05):
    global DEFAULT_ROTATION_SPEED, RUNNING

    current = 0
    final = 0
    BrickPiUpdateValues()
    BrickPi.MotorEnable[PORT_D] = 1
    
    #For running clockwise and anticlockwise
    if deg > 0:
        BrickPi.MotorSpeed[PORT_D] = DEFAULT_ROTATION_SPEED
    elif deg < 0:
        BrickPi.MotorSpeed[PORT_D] = -DEFAULT_ROTATION_SPEED
    else:
        BrickPi.MotorSpeed[PORT_D] = 0

    current = BrickPi.Encoder[PORT_D]  # Initial value of the encoder
    final = deg*2  # Final value of the encoder
    done = False

    while RUNNING:
        result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        if not result:
            # Check if final value reached for each of the motors
            if (deg>0 and final>current) or (deg<0 and final<current):
                # Read the encoder degrees
                current=BrickPi.Encoder[PORT_D]
            else:
                done = True
                if deg > 0:
                    BrickPi.MotorSpeed[PORT_D] = -DEFAULT_ROTATION_SPEED
                elif deg < 0:
                    BrickPi.MotorSpeed[PORT_D] = DEFAULT_ROTATION_SPEED
                else:
                    BrickPi.MotorSpeed[PORT_D] = 0
                BrickPiUpdateValues()
                time.sleep(delay_when_stopping)
                BrickPi.MotorEnable[PORT_D] = 0
                BrickPiUpdateValues()
        time.sleep(sampling_time)  # Sleep for the sampling time given (default:100 ms)
        if done:  # If all the motors have already completed their rotation, then stop
            break
    return 0


def sense():
    global RIGHT_WHEEL, LEFT_WHEEL, ANGLE, FORWARD, BACKWARD
    RIGHT_WHEEL = BrickPi.Encoder[PORT_B]
    LEFT_WHEEL = BrickPi.Encoder[PORT_C]
    ANGLE = BrickPi.Encoder[PORT_D]
    FORWARD = BrickPi.Sensor[PORT_2]
    BACKWARD = BrickPi.Sensor[PORT_3]
    return [RIGHT_WHEEL, LEFT_WHEEL, ANGLE, FORWARD, BACKWARD]


def forward(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = speed
    BrickPi.MotorSpeed[PORT_C] = speed


def backward(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = -speed
    BrickPi.MotorSpeed[PORT_C] = -speed


def right(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = -speed
    BrickPi.MotorSpeed[PORT_C] = speed


def left(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = speed
    BrickPi.MotorSpeed[PORT_C] = -speed


def stop(speed=DEFAULT_MOVEMENT_SPEED):
    BrickPi.MotorSpeed[PORT_B] = 0
    BrickPi.MotorSpeed[PORT_C] = 0


def scan():
    global RUNNING
    while RUNNING:
        rotate_sensor(90)
        rotate_sensor(-90)
    return 0


def main():
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
            stop()
            RUNNING = False
            SOCK.close()
            break

        BrickPiUpdateValues()
        state = sense()
        SOCK.sendto(",".join([str(val) for val in state]), (UDP_IP, UDP_PORT))

        time.sleep(0.1)
    return 0

threading.Thread(target=scan).start()
threading.Thread(target=main).start()