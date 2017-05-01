"""Module for storing and updating the robot state."""

import time
import numpy as np

from server import util


class State(object):
    """Class which stores information about robot state/adjustment.

    Attributes:
        location (np.array): State location.
        heading (float): Heading direction.
    """

    def __init__(self, location=np.zeros(2), heading=0.0):
        """Initialise state object.
        
        Args:
            location (np.array): State location.
            heading (float): Heading direction.
        """
        self.location = location
        self.heading = heading

    def __add__(self, other):
        """Add two states by adding the individual components.
        
        Args:
            other (State): State to add to this one.

        Returns:
            State: Added states.
        """
        return State(location=self.location+other.location, heading=self.heading+other.heading)

    def update(self, location, heading):
        """Set the location and direction of the state.
        
        Args:
            location (np.array): New location.
            heading (float): New heading direction.
        """
        self.location = location
        self.heading = heading

    def delta(self, delta_location=np.zeros(2), delta_heading=0.0):
        """Update the location and heading.

        Args:
            delta_location (np.array): Change in location.
            delta_heading (float): Change in angle.
        """
        self.location += delta_location
        self.heading += delta_heading


class Measurement(object):
    """Class which stores measurement info, along with the state of the robot when the measurement was taken.
    
    Attributes:
        state (State): State of the robot.
        angle (float): Angle of the sensor.
        distance (float): Distance sensed by sensor.
        location (np.array): Location of the sensed object.
    """

    def __init__(self, state, angle, distance):
        """Initialise measurement object.
        
        Args:
            state (State): State of the robot.
            angle (float): Angle of the sensor.
            distance (float): Distance sensed by sensor.
        """
        self.timestamp = time.time()
        self.state = state
        self.angle = angle
        self.distance = distance
        self.location = self.state.location + util.pol_to_cart(self.distance, self.angle + self.state.heading)


class Bot(object):
    """Main class for the robot.
    
    Attributes:
        state (State): Raw measured state.
        adjustment (State): SLAM adjustment.
        adjusted (State): State combined with adjustment.
    """
    def __init__(self):
        """Initialise robot object."""
        self.state = State()
        self.adjustment = State()
        self.adjusted = self.state + self.adjustment

    # noinspection PyListCreation
    def update(self, observation):
        """Access the measurements recieved since the last sense, update the robot's state, and return them as a list of
        Measurement objects.

        Returns:
            list: List of measurements.
        """
        x, y, heading, angle, front, rear = observation

        #  Need to update adjustment if the angle has changed due to SLAM.
        location = np.array([x, y])  # Convert to np.array.
        change = location - self.state.location  # Change in location.
        adjusted = util.rotate_point(np.zeros(2), change, self.adjustment.heading)  # Adjusted change in location
        delta = adjusted - change  # Change in adjustment.

        # Update the state and adjustment
        self.state.update(location, heading)
        self.adjustment.delta(delta)
        self.adjusted = self.state + self.adjustment

        # Append measurements for front and rear sensors.
        measurements = []
        measurements.append(Measurement(self.adjusted, (angle/2) % 360, front))
        measurements.append(Measurement(self.adjusted, (angle/2 + 180) % 360, rear))
        return measurements
