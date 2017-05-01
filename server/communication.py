"""Module for communicating with the Raspberry Pi."""

import socket
import threading
import time
import numpy

from server import util
from server.robot import Measurement


class Comm(threading.Thread):
    """Class for communicating with Raspberry Pi.

    Attributes:
        running (bool): True when the communication thread is running.
        measurements (list): Measurements received since last sensed.
        robot (robot.Bot): Robot object.
        udp_ip (str) = The IP of the Raspberry Pi.
        udp_port (int) = The port of the Raspberry Pi.
        sock (socket.socket) = The socket connecting to the Raspberry Pi.
    """
    def __init__(self, robot, size=0):
        """Initialise Communication object."""
        threading.Thread.__init__(self)

        self.running = True
        self.measurements = []
        self.robot = robot

        # Socket variables
        self.udp_ip = "192.168.137.238" # Change to Raspberry Pi's IP.
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.udp_port))

    def run(self):
        """Thread loop."""
        while self.running:
            data, address = self.sock.recvfrom(1024)  # Get data from socket.
            decoded = data.decode().split(',')  # Split the data.
            observation = [float(x) for x in decoded]  # Extract numerical values.
            self.measurements.extend(self.robot.update(observation))  # Update the robot with the observation.
        self.sock.close()

    def get_measurements(self):
        """Get measurements since last time sensing."""
        temp = list(self.measurements)
        self.measurements = []
        return temp

    def get_median_measurements(self):
        """Return the median of the readings in the 20 degree range about each angle, from 0 to 360."""
        measurements = self.get_measurements()
        result = []
        for i in range(360):
            # Get the measurements in a 20 degree range.
            close_measurements = [x.distance for x in measurements if util.angle_diff(x.angle, i) < 10]
            if len(close_measurements) > 0:
                # Get the median, and append the measurement
                result.append(Measurement(self.robot.adjusted, i, util.middle(close_measurements)))

        return result

    def stop(self):
        """Send stop instruction to robot, and stop the thread."""
        self.sock.sendto("STOP".encode(), (self.udp_ip, self.udp_port))
        self.running = False

    def move(self, power, rotate):
        """Send movement instructions to robot.

        Args:
            power (int): Robot speed in range [-255, 255]
            rotate (bool): Rotating or moving in straight line.
        """
        self.sock.sendto((str(power) + "," + str(rotate)).encode(), (self.udp_ip, self.udp_port))

    def pause(self):
        """Send pause robot spinning command."""
        self.sock.sendto("PAUSE".encode(), (self.udp_ip, self.udp_port))

    def resume(self):
        """Send resume robot spinning command."""
        self.sock.sendto("RESUME".encode(), (self.udp_ip, self.udp_port))

    def turn(self, angle):
        """Turn robot a specified angle."""
        start = self.robot.adjusted.heading
        self.move(numpy.sign(angle)*100, True)

        # Keep turning while not reached angle.
        while util.angle_diff(start, self.robot.adjusted.heading) < angle:
            time.sleep(0.02)
        self.move(0, False)

    def drive(self, distance):
        """Move forward a specified distance."""
        start = self.robot.adjusted.location
        self.move(numpy.sign(distance)*255, False)

        # Keep moving while not moved full distance.
        while util.dist(self.robot.adjusted.location, start) < distance:
            time.sleep(0.02)
        self.move(0, False)
