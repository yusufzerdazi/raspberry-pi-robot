"""Module which simulates the function of the Raspberry Pi robot."""

import time
import math
import numpy as np
from server import util
import random
import threading
from server.robot import State, Measurement


class Comm(threading.Thread):
    """Class for simulating response from a Raspberry Pi.

        Attributes:
            running (bool): True when the communication thread is running.
            measurements (list): Measurements received since last sensed.
            robot (robot.Bot): Robot object.
            alternative: position of the robot with error.
            actual: Actual position of the robot.
            distance_error: The extra amount the robot moves in a straight line.
            heading_error: The extra amount the robot turns.
            drift_error: The amount the robot drifts to the right.
            heading_drift_error: The amount the robot drifts rotationally.
            current: The current time.
            rotating: Whether the robot is rotating or not.
            spinning: Whether the sensor is spinning or not.
            speed: The speed of the robot.
            measurements: An array of measurements the robot has made.
            size: The size of the room the robot is in.
        """
    def __init__(self, robot, distance_error=0, heading_error=0, drift_error=0, heading_drift_error=0, size=100):
        """Initialise the simulation object.
        
        Args:
            robot: The robot object
            distance_error: The straight line error.
            heading_error: The rotational error.
            drift_error: The amount the robot drifts to the right.
            heading_drift_error: The amount the robot turns.
            size: The size of the room.
        """
        threading.Thread.__init__(self)

        # Actual robot state
        self.robot = robot

        # Variables to keep track of state.
        self.alternative = State()
        self.actual = State()

        # Error variables.
        self.distance_error = distance_error
        self.heading_error = heading_error
        self.drift_error = drift_error
        self.heading_drift_error = heading_drift_error

        # Time for calculating distance moved.
        self.current = time.time()

        # Variables for moving/rotating robot.
        self.rotating = False
        self.spinning = True
        self.speed = 0
        self.measurements = []
        self.running = True

        # Size of room.
        self.size = size

    def run(self):
        """Running loop."""
        while self.running:
            # Only update once every 0.02 seconds maximum.
            if time.time() - self.current < 0.02:
                time.sleep(0.02)

            # Calculate time difference.
            new = time.time()
            delta_time = new - self.current

            # Rotate/move and add the error.
            if self.rotating:
                delta_heading = self.speed * delta_time * 0.4
                error_delta_heading = delta_heading + self.heading_error
                self.alternative.heading += delta_heading
                self.actual.heading += error_delta_heading
            else:
                distance = self.speed * delta_time * 0.1
                self.actual.heading += self.heading_drift_error * distance
                error_distance = distance + self.distance_error * distance
                delta_position = np.array([distance * math.cos(math.radians(self.alternative.heading)),
                                           distance * math.sin(math.radians(self.alternative.heading))])
                error_delta_position = np.array([error_distance * math.cos(math.radians(self.actual.heading)) +
                                                 delta_time * bool(self.speed) * self.drift_error,
                                                 error_distance * math.sin(math.radians(self.actual.heading))])
                self.alternative.location = np.array([self.alternative.location[0] + delta_position[0],
                                                      self.alternative.location[1] + delta_position[1]])
                self.actual.location = np.array([self.actual.location[0] + error_delta_position[0],
                                                 self.actual.location[1] + error_delta_position[1]])

            # Calculate the angle of the sensor
            t = self.current
            m = 180
            angle = m - abs(t * 150 % (2 * m) - m) - 90

            # Calculate the distance value returned by the sensors.
            RY = self.size
            RX = self.size
            if 90 < (angle + self.actual.heading) % 360 <= 270:
                FA = abs((RX + self.actual.location[0]) / math.cos(math.radians(angle + self.actual.heading))) + 4 * random.random()
                RA = abs((RX - self.actual.location[0]) / math.cos(math.radians(angle + self.actual.heading))) + 4 * random.random()
            else:
                FA = abs((RX - self.actual.location[0]) / math.cos(math.radians(angle + self.actual.heading))) + 4 * random.random()
                RA = abs((RX + self.actual.location[0]) / math.cos(math.radians(angle + self.actual.heading))) + 4 * random.random()

            if 180 < (angle + self.actual.heading) % 360 <= 360:
                FB = abs((RY + self.actual.location[1]) / math.sin(math.radians(angle + self.actual.heading))) + 4 * random.random()
                RB = abs((RY - self.actual.location[1]) / math.sin(math.radians(angle + self.actual.heading))) + 4 * random.random()
            else:
                FB = abs((RY - self.actual.location[1]) / math.sin(math.radians(angle + self.actual.heading))) + 4 * random.random()
                RB = abs((RY + self.actual.location[1]) / math.sin(math.radians(angle + self.actual.heading))) + 4 * random.random()
            F = min(FA, FB)
            R = min(RA, RB)
            front = F if (F < 100) else 255
            rear = R if (R < 100) else 255

            # Append the measurements.
            self.measurements.extend(self.robot.update((self.alternative.location[0], self.alternative.location[1], self.alternative.heading, (angle * 2), front, rear)))
            self.current = new

    def get_measurements(self):
        """Access the measurements recieved since the last sense, update the robot's state, and return
        them as a list of Measurement objects.

        Returns:
            list: List of measurements.
        """
        result = list(self.measurements)
        self.measurements = []
        return result

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

    def move(self, speed, rotate):
        """Sends speed and direction instructions to robot"""
        self.current = time.time()
        self.speed = speed
        self.rotating = rotate

    def stop(self):
        """Tells robot to stop"""
        self.running = False

    def pause(self):
        """Tells robot to pause sensing"""
        self.spinning = False

    def resume(self):
        """Tells robot to resume sensing"""
        self.spinning = True

    def turn(self, angle):
        start = self.robot.adjusted.heading
        self.move(np.sign(angle)*180, True)
        while util.angle_diff(start, self.robot.adjusted.heading) < abs(angle):
            time.sleep(0.02)
        self.move(0, False)

    def drive(self, distance):
        start = self.robot.adjusted.location
        self.move(np.sign(distance)*100, False)
        while util.dist(self.robot.adjusted.location, start) < distance:
            time.sleep(0.02)
        self.move(0, False)