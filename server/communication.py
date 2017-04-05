import socket
import threading
import time
import numpy
import copy

from server import util
from server.view import bayesian_estimation

# Socket variables
UDP_IP = "192.168.137.29"
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
SOCK.bind(("", UDP_PORT))


class Comm(threading.Thread):
    """Class for communicating with Raspberry Pi.

    Attributes:
        running (bool): True when the socket is open.
        measurements (list): Measurements received since last sensed.
    """
    def __init__(self, robot):
        """Initialise Communication object."""
        threading.Thread.__init__(self)

        self.running = True
        self.measurements = []
        self.robot = robot

    def run(self):
        """Thread loop."""
        while self.running:
            data, address = SOCK.recvfrom(1024)  # Get data from socket.
            observation = data.decode().split(',')  # Split the data.
            self.measurements.extend(self.robot.update(observation))
        SOCK.close()

    def get_measurements(self):
        """Get measurements since last time sensing."""
        temp = list(self.measurements)
        self.measurements = []
        return temp

    def stop(self):
        """Send stop instruction to robot."""
        SOCK.sendto("STOP".encode(), (UDP_IP, UDP_PORT))
        self.running = False

    def move(self, power, rotate):
        """Send movement instructions to robot.

        Args:
            power (int): Robot speed in range [-255, 255]
            rotate (bool): Rotating or moving in straight line.
        """
        SOCK.sendto((str(power) + "," + str(rotate)).encode(), (UDP_IP, UDP_PORT))

    def pause(self):
        """Send pause robot spinning command."""
        SOCK.sendto("PAUSE".encode(), (UDP_IP, UDP_PORT))

    def resume(self):
        """Send resume robot spinning command."""
        SOCK.sendto("RESUME".encode(), (UDP_IP, UDP_PORT))

    def turn(self, angle):
        """Turn robot a specified angle."""
        start = self.robot.adjusted.heading
        self.move(numpy.sign(angle)*180, True)
        while util.angle_diff(start, self.robot.adjusted) < angle:
            time.sleep(0.02)
        self.move(0, False)

    def drive(self, distance):
        """Move forward a specified distance."""
        start = self.robot.adjusted.location
        self.move(numpy.sign(distance)*100, False)
        while util.dist(self.robot.adjusted.location, start) < distance:
            time.sleep(0.02)
        self.move(0, False)
