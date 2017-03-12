import math
import numpy as np

import communication
import algebra


class State(object):
    def __init__(self, location=np.zeros(2), heading=0.0):
        self.location = location
        self.heading = heading

    def __add__(self, other):
        return State(location=self.location+other.location, heading=self.heading+other.heading)

    def update(self, location, heading):
        self.location = location
        self.heading = heading

    def delta(self, delta_location=np.zeros(2), delta_heading=0.0):
        self.location += delta_location
        self.heading += delta_heading


class Measurement(object):
    def __init__(self, state, angle, distance):
        self.state = state
        self.angle = angle
        self.distance = distance
        self.location = self.state.location + np.array([self.distance*math.cos(math.radians(self.angle+self.state.heading)),
                                                        self.distance*math.sin(math.radians(self.angle+self.state.heading))])


class Robot(object):
    def __init__(self):
        self.state = State()
        self.adjustment = State()
        self.adjusted = self.state + self.adjustment
        self.communication = communication.Communication()
        self.communication.start()

    def sense(self):
        """Access the measurements recieved since the last sense, update the robot's state, and return
        them as a list of Measurement objects.

        Returns:
            list: List of measurements.
        """
        measurements = []
        sensed = self.communication.sense()

        # For each measurement.
        for x, y, heading, angle, front, rear in sensed:
            # If there is a heading adjustment, the recieved coordinates need to be adjusted.
            location = np.array([x, y])
            moved = location - self.state.location
            delta = algebra.rotate_point(np.zeros(2), moved, self.adjustment.heading) - moved

            # Update the state and adjustment
            self.state.update(location, heading)
            self.adjustment.delta(delta)
            self.adjusted = self.state + self.adjustment

            # Append measurements for front and rear sensors.
            measurements.append(Measurement(self.adjusted, (angle/2) % 360, front))
            measurements.append(Measurement(self.adjusted, (angle/2 + 180) % 360, rear))

        return measurements

    def senses(self, n):
        measurements = []
        while len(measurements) < n:
            new = self.sense()
            measurements.extend([m for m in new if m.distance not in [-1, 255]])
        return measurements

    def move(self, speed, rotate):
        """Sends speed and direction instructions to robot"""
        self.communication.move(speed, rotate)

    def stop(self):
        """Tells robot to stop"""
        self.communication.stop()

    def pause(self):
        """Tells robot to pause sensing"""
        self.communication.pause()

    def resume(self):
        """Tells robot to resume sensing"""
        self.communication.resume()
