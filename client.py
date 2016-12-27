#!/usr/bin/env python
# Yusuf Zerdazi

from BrickPi import *
import socket
import threading
import time

UDP_IP = "<IP>"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP

BrickPiSetup()  # setup the serial port for communication

# Enable the motors
BrickPi.MotorEnable[PORT_B] = 1
BrickPi.MotorEnable[PORT_C] = 1
BrickPi.MotorEnable[PORT_D] = 1

BrickPi.MotorSpeed[PORT_B] = 0
BrickPi.MotorSpeed[PORT_C] = 0
BrickPi.MotorSpeed[PORT_D] = 0

# Enable the ultrasonic sensors
BrickPi.SensorType[PORT_2] = TYPE_SENSOR_ULTRASONIC_CONT
BrickPi.SensorType[PORT_3] = TYPE_SENSOR_ULTRASONIC_CONT

# Setup the sensors
BrickPiSetupSensors()
time.sleep(5)
BrickPiUpdateValues()
time.sleep(5)

B_INITIAL = BrickPi.Encoder[PORT_B]
C_INITIAL = BrickPi.Encoder[PORT_C]
D_INITIAL = BrickPi.Encoder[PORT_D]

print B_INITIAL
print C_INITIAL
print D_INITIAL

DISTANCE_PER_DEGREE = 18.0 / 360.0


class Robot(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0

        self.forward = -1
        self.backward = -1

        self.max_speed = 50
        self.spin_speed = 40
        self.direction = -1

    def start(self):
        BrickPi.MotorSpeed[PORT_B] = self.max_speed
        BrickPi.MotorSpeed[PORT_C] = self.max_speed

    def drive(self):
        angles = self.get_motor_angles()
        robot_distance = (angles[0] + angles[1]) / 2
        self.y = DISTANCE_PER_DEGREE * robot_distance

    def stop(self):
        BrickPi.MotorSpeed[PORT_B] = 0
        BrickPi.MotorSpeed[PORT_C] = 0
        BrickPi.MotorSpeed[PORT_D] = 0

    def spin(self):
        BrickPi.MotorSpeed[PORT_D] = self.spin_speed * self.direction

    def get_sensor_angle(self):
        result = BrickPiUpdateValues()
        if not result:
            return (BrickPi.Encoder[PORT_D] - D_INITIAL) / 2
        else:
            return -1

    def get_motor_angles(self):
        BrickPiUpdateValues()
        return (BrickPi.Encoder[PORT_B] - B_INITIAL) / 2, (BrickPi.Encoder[PORT_C] - C_INITIAL) / 2

    def sense(self):
        BrickPiUpdateValues()

        result = []
        angle = self.get_sensor_angle()
        self.forward = BrickPi.Sensor[PORT_2]
        self.backward = BrickPi.Sensor[PORT_3]

        if self.forward == -1:
            self.forward = 255
        if self.backward == -1:
            self.backward = 255

        result.append((self.forward, angle % 360))
        result.append((self.backward, (angle + 180) % 360))

        return result

    def send_reading(self):
        distances = self.sense()
        self.drive()
        for distance, angle in distances:
            sock.sendto(str(distance) + ", " + str(angle) + ", " + str(self.x) + ", " + str(self.y), (UDP_IP, UDP_PORT))


robot = Robot()
robot.start()
robot.spin()
count_1 = 0
count_2 = 0

while True:
    angle = robot.get_sensor_angle()
    if angle < -90:
        robot.direction = 1
        robot.spin()
    elif angle > 90:
        robot.direction = -1
        robot.spin()
    print angle
    print robot.forward
    print robot.backward
    print ""
    if (-10 <= angle <= 10) and ((0 <= robot.backward <= 40) or (0 <= robot.forward <= 40)):
        print "STOPPING!!!"
        robot.stop()
        break
    robot.send_reading()
    time.sleep(0.15)
