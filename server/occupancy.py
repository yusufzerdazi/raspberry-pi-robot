"""Module for storing the occupancy grid of the map, as well as preliminary calculations."""

import math

import numpy as np
from PIL import Image, ImageDraw

from server.util import ViewMode, ProbabilityMode

SENSOR_MEAN = 100.0148733
SENSOR_STDDEV = 3.685631181


class Grid(object):
    """Class for the occupancy grid, composed of a number of PIL images, for tracking objects and landmarks.

    Attributes:
        width (int): Width of image.
        height (int): Height of image.
        scanning (bool): Whether scans are being received for SLAM.
        probabilities (bool): Determines whether to show probabilities or map.
        view_mode (ViewMode): Determining whether to show the map with or without adjustment.
        probability_mode (ProbabilityMode): Determines which probability calculation stage to display.
        origin (np.ndarray): The centre of the image, where the robot starts.
        colours (dict): Colour of unadjusted and adjusted states.

        display (PIL.Image): The merged image to display.
        view_images (dict): Dict containing occupancy grids.
        probability_images (dict): Dict containing images for probabilities.

    """
    def __init__(self, width, height):
        """Initialise occupancy grid.

        Args:
            width: Width of grid.
            height: Height of grid.
        """
        # Width and height
        self.width = width
        self.height = height

        # Selected view and probability modes.
        self.view_mode = ViewMode.ADJUSTED
        self.probability_mode = ProbabilityMode.COMBINED_PROBABILITIES

        # Origin location and main display image.
        self.origin = np.array([self.width/2, self.height/2])
        self.display = Image.new("RGBA", (self.width, self.height), "gray")
        self.trail = Image.new("RGBA", (self.width, self.height), (0, 0, 0, 0))

        # Variables for showing probabilities and scanning. Will alter what is displayed on screen.
        self.probabilities = False
        self.scanning = False

        # Colours of displayed robot states.
        self.colours = {
            ViewMode.ADJUSTED: "red",
            ViewMode.STATE: "blue"
        }

        # Occupancy grids
        self.view_images = {
            ViewMode.ADJUSTED: Image.new("RGBA", (self.width, self.height), "gray"),
            ViewMode.STATE: Image.new("RGBA", (self.width, self.height), "gray"),
            ViewMode.LOCAL: Image.new("RGBA", (self.width, self.height), "gray")
        }

        # Probability grids
        self.probability_images = {
            ProbabilityMode.COMBINED_PROBABILITIES: Image.new("RGBA", (self.width, self.height), "white"),
            ProbabilityMode.SLAM_PROBABILITIES: Image.new("RGBA", (self.width, self.height), "white"),
            ProbabilityMode.PRIOR_PROBABILITIES: Image.new("RGBA", (self.width, self.height), "white"),
            ProbabilityMode.GLOBAL_MAP:  Image.new("RGBA", (self.width, self.height), "black"),
            ProbabilityMode.LOCAL_MAP: Image.new("RGBA", (self.width, self.height), "black"),
        }

    def plot_measurement(self, measurement, radius=2, naive=False):
        """Plot a measurements.

        Args:
            measurement (Measurement): Measurement to plot.
            radius (int): Radius of measurement ends.
        """
        if naive:
            draw = ImageDraw.Draw(self.view_images[ViewMode.ADJUSTED])
        else:
            draw = ImageDraw.Draw(self.view_images[ViewMode.LOCAL])

        # Add origin so it get's displayed from centre.
        start = measurement.state.location + self.origin
        end = measurement.location + self.origin

        # Bounding box for measurement
        measurement_box = ((end[0]-radius, end[1]-radius), (end[0]+radius, end[1]+radius))

        # Plot measurement.
        scan_colour = (80, 80, 80)
        point_colour = (170, 170, 170)
        draw.line(((start[0], start[1]), (end[0], end[1])), fill=scan_colour)
        draw.ellipse(measurement_box, fill=point_colour)

    def plot_state(self, state, state_type, radius=3):
        """Plot the location and direction of the robot.

        Args:
            state (State): Robot state.
            state_type (ViewMode): Which state to plot.
            radius (int): Radius of robot.
        """
        draw = ImageDraw.Draw(self.display)

        # Specify how to display the state.
        colour = self.colours[state_type]
        if state_type == ViewMode.STATE:
            length = 50
        else:
            length = 25

        # Calculate the x and y lengths for the robot direction
        end_x = self.origin[0] + state.location[0] + length * math.cos(math.radians(state.heading))
        end_y = self.origin[1] + state.location[1] + length * math.sin(math.radians(state.heading))

        # Bounding box for robot
        robot_box = (tuple(state.location + self.origin - radius), tuple(state.location + self.origin + radius))

        # Plot location and direction.
        draw.ellipse(robot_box, fill=colour)
        draw.line((tuple(state.location + self.origin), (end_x, end_y)), fill=colour)

        # Clockwise part of arrow head
        cw_x = end_x + 10 * math.cos(math.radians(state.heading + 135))
        cw_y = end_y + 10 * math.sin(math.radians(state.heading + 135))

        # Counterclockwise part of arrow head
        ccw_x = end_x + 10 * math.cos(math.radians(state.heading - 135))
        ccw_y = end_y + 10 * math.sin(math.radians(state.heading - 135))

        # Plot arrow head
        draw.line((end_x, end_y, cw_x, cw_y), fill=colour)
        draw.line((end_x, end_y, ccw_x, ccw_y), fill=colour)

    def plot_trail(self, current, state_type):
        """Plot the current location of the robot as a single pixel.

        Args:
            current (State): Current robot state.
            state_type (ViewMode): View mode of state.
        """
        colour = self.colours[state_type]
        trail = ImageDraw.Draw(self.trail)
        trail.point(tuple((current.location+self.origin).astype(int)), fill=colour)

    def plot_landmark(self, landmark, colour="black"):
        """Plot a landmark as a line segment.

        Args:
            landmark (Landmark): Landmark to plot.
            colour (tuple): Colour to plot the landmark
        """
        if landmark.association:
            colour = "red"
        draw = ImageDraw.Draw(self.display)
        draw.line((tuple((self.origin+landmark.segment[0]).astype(int)),
                   tuple((self.origin+landmark.segment[1]).astype(int))), fill=colour)

    def plot_landmarks(self, landmarks, colour="black"):
        """Plot a  list of landmarks as a line segment.

        Args:
            landmarks (list): Landmarks to plot.
            colour (tuple): Colour to plot the landmark
        """
        for landmark in landmarks:
            self.plot_landmark(landmark, colour)

    def plot_prob_dist(self, dist, dist_type, normalise=True):
        """Given a dictionary mapping coordinates to probability values, plot this distribution, where darker values
        represent more likely locations.

        Args:
            dist (dict): Probability distribution.
            dist_type (server.util.ProbabilityMode): Probability mode.
            normalise (bool): Normalise the values so smallest show up as white and largest as black.
        """
        draw = ImageDraw.Draw(self.probability_images[dist_type])

        # Variables for normalisation.
        max_prob = 1
        min_prob = 0

        # Determine whether normalisation can be performed.
        values = list(dist.values())
        if len(values) > 0 and sum(values) > 0:
            max_prob = max(values)
            if normalise and min(values) != max_prob:
                min_prob = min(values)
            else:
                return -1

        # Plot probability values.
        for key in dist:
            value = int(255 - 255 * ((dist[key] - min_prob) / (max_prob - min_prob)))
            if self.probability_images[dist_type].getpixel(tuple(self.origin + np.array(key)))[0] > value:
                draw.point(tuple(self.origin + np.array(key)), fill=(value, value, value))

    def clear(self):
        """Clear the images."""
        self.view_images[ViewMode.LOCAL] = Image.new("RGBA", (self.width, self.height), "gray")

    def combine(self, robot, landmarks):
        """Combine the images with state and landmarks.

        Args:
            robot (robot.Robot): Robot state.
            landmarks (list): List of landmarks.
        """
        if self.probabilities:
            return self.probability_images[self.probability_mode]
        else:
            self.display = self.view_images[self.view_mode].copy()
            self.plot_landmarks(landmarks)
            self.display = Image.alpha_composite(self.display, self.trail)
            self.plot_state(robot.state, self.view_mode.STATE)
            self.plot_state(robot.adjusted, self.view_mode.ADJUSTED)
            return self.display


def translate(image, angle, center=None, new_center=None, scale=None):
    # http://stackoverflow.com/questions/7501009/affine-transform-in-pil-python
    if center is None:
        return image.rotate(angle)
    angle = -angle / 180.0 * math.pi
    nx, ny = x, y = center
    sx = sy = 1.0
    if new_center is not None:
        (nx, ny) = new_center
    if scale:
        (sx, sy) = scale
    cosine = math.cos(angle)
    sine = math.sin(angle)
    a = cosine / sx
    b = sine / sx
    c = x - nx * a - ny * b
    d = -sine / sy
    e = cosine / sy
    f = y - nx * d - ny * e
    return image.transform(image.size, Image.AFFINE, (a, b, c, d, e, f))


def black_white(image):
    """Convert to black and white."""
    i = np.array(image.convert("L"))
    return (i > 150) * 255
