import sys
import math
import threading
import enum

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt

#from server import communication
import server.robot_simulator as communication
from server import robot
from server import slam


WIDTH = 2000
HEIGHT = 2000
SPEED = 180


class TrackingMode(enum.Enum):
    FREE = 0
    STATE = 1
    ADJUSTED = 2


class ViewMode(enum.Enum):
    STATE = 0
    ADJUSTED = 1


class ProbabilityMode(enum.Enum):
    PROBABILITIES = 0
    SLAM_PROBABILITIES = 1
    PRIOR_PROBABILITIES = 3


class MapViewer(QtWidgets.QMainWindow):
    """Main class containing robot and for updating it's generated map.
    
    Attributes:
        map (Map): Map object.
        robot (robot.Robot): Robot object
        slam (slam.Slam): Slam object
        view_scale (float): Zoom factor for map.
        controlled (bool): Keyboard or automatic robot movement.
        image (PIL.Image): Image to display on screen.
        distribution (PIL.Image): Probability distribution history from SLAM.
    """
    def __init__(self, controlled=True):
        """Initialise MapViewer object.

        Args:
            controlled: Whether the robot is controlled via input or automatically.
        """
        super(MapViewer, self).__init__()

        # Map attributes

        self.controlled = controlled  # Remote controlled.
        self.distribution = False  # Whether to display probability distribution or map.

        # Robot attributes.
        self.map = Map(WIDTH, HEIGHT)  # Initialise map.
        self.image = self.map.view_images[ViewMode.ADJUSTED]  # Displayed image.
        self.robot = robot.Robot()  # Initialise robot.


        self.communication = communication.Communication(self.robot, self.map)
        self.communication.start()

        self.slam = slam.Slam(self.robot, self.map, self.communication, self.controlled)  # Initialise SLAM.
        self.slam.start()  # Start SLAM thread.

        # Set up the window
        self.imageLabel = QtWidgets.QLabel()
        self.imageLabel.setBackgroundRole(QtGui.QPalette.Base)
        self.imageLabel.setScaledContents(True)

        self.scrollArea = QtWidgets.QScrollArea()
        self.scrollArea.setWidget(self.imageLabel)
        self.setCentralWidget(self.scrollArea)

        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)  # Disable horizontal scrollbar.
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)  # Disable vertical scrollbar.

        self.setWindowTitle("Robot Map")  # Set title.
        self.showFullScreen()  # Make fullscreen.

        self.dimensions = np.array([self.scrollArea.frameGeometry().width(),
                                    self.scrollArea.frameGeometry().height()])
        self.view_centre = np.array([0,0])
        self.view_scale = 5.0
        self.mouse_pos = self.view_centre
        self.tracking_mode = TrackingMode.FREE

        # Set up image refresh timer.
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.display)
        timer.start(33)  # 30 Hz

        threading.Thread(target=self.crop).start()

    def update(self):
        observations = self.communication.sense()
        self.robot.update(observations)
        self.map.plot_trail(self.robot.state, (0,0,255,127))
        self.map.plot_trail(self.robot.adjusted, (255,0,0,127))

    def crop(self):
        while True:
            """Combine sub-images, and crop based on the scale factor."""
            if self.tracking_mode == TrackingMode.ADJUSTED:
                self.view_centre = self.robot.adjusted.location
            elif self.tracking_mode == TrackingMode.STATE:
                self.view_centre = self.robot.state.location
            size = self.dimensions / (self.view_scale * 2)  # Size of cropped window.
            crop_box = tuple(np.append(self.map.origin + self.view_centre - size,
                                       self.map.origin + self.view_centre + size))  # Crop box
            image = self.map.combine(self.robot, self.slam.landmarks)
            self.image = image.crop(crop_box).resize(self.dimensions)  # Update image.

    def display(self):
        """Convert PIL to ImageQt, and display on screen."""
        image_qt = ImageQt.ImageQt(self.image)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

    def wheelEvent(self, event):
        """Zoom in/out"""
        degrees = event.angleDelta().y() / 8
        steps = degrees / 15
        self.view_scale *= 1.5 ** steps

    def mouseMoveEvent(self, event):
        """Allow dragging of the map to move the view window, when free tracking is enabled."""
        if self.tracking_mode == TrackingMode.FREE and event.buttons() == QtCore.Qt.LeftButton:

            # Calculate the change in mouse position.
            new_mouse_pos = np.array([event.x(), event.y()])
            mouse_delta = new_mouse_pos - self.mouse_pos

            # Add this to the view centre.
            self.view_centre = self.view_centre - mouse_delta * (1 / self.view_scale)
            self.mouse_pos = new_mouse_pos

    def mousePressEvent(self, event):
        """Set the mouse start location, so movement delta can be calculated."""
        if event.buttons() == QtCore.Qt.LeftButton:
            self.mouse_pos = np.array([event.x(), event.y()])

    def keyPressEvent(self, event):
        """Detect when a key is pressed, and perform the relevent function.
        
        Args:
            event: KeyPress event.
        """
        key = event.key()  # Get the key.

        # Exit the application.
        if key == QtCore.Qt.Key_Escape:
            self.close()
            self.slam.stop()
            self.slam.resume()
            self.communication.stop()
        # Clear the screen.
        elif key == QtCore.Qt.Key_Q:
            self.map.clear()
        elif key == QtCore.Qt.Key_1:
            self.tracking_mode = TrackingMode.FREE
        elif key == QtCore.Qt.Key_2:
            self.tracking_mode = TrackingMode.ADJUSTED
        elif key == QtCore.Qt.Key_3:
            self.tracking_mode = TrackingMode.STATE

        elif key == QtCore.Qt.Key_8:
            self.map.probability_mode = ProbabilityMode.SLAM_PROBABILITIES
        elif key == QtCore.Qt.Key_9:
            self.map.probability_mode = ProbabilityMode.PRIOR_PROBABILITIES
        elif key == QtCore.Qt.Key_0:
            self.map.probability_mode = ProbabilityMode.PROBABILITIES

        # Toggle between probability distribution and map.
        elif key == QtCore.Qt.Key_P:
            self.map.probabilities = not self.map.probabilities

        # Remote control commands.
        elif self.controlled:
            # Move forward.
            if key == QtCore.Qt.Key_W:
                self.communication.move(SPEED, False)
            # Turn left.
            elif key == QtCore.Qt.Key_A:
                self.communication.move(-SPEED, True)
            # Move backward.
            elif key == QtCore.Qt.Key_S:
                self.communication.move(-SPEED, False)
            # Turn right.
            elif key == QtCore.Qt.Key_D:
                self.communication.move(SPEED, True)
            # Stop movement.
            elif key == QtCore.Qt.Key_Z:
                self.communication.move(0, False)

            # Pause spinning.
            elif key == QtCore.Qt.Key_X:
                self.communication.pause()
            # Resume spinning.
            elif key == QtCore.Qt.Key_C:
                self.communication.resume()

            # Perform SLAM iteration.
            elif key == QtCore.Qt.Key_Return:  
                self.slam.resume()


class Map(object):
    """Map class, composed of a number of PIL images, for tracking objects and landmarks.

    Attributes:
        state (PIL.Image): Grey image for background.
        objects (PIL.Image): Transparent image for obstacles.
        space (PIL.Image): Transparent image for detected free space.
        state (PIL.Image): Transparent image for raw robot state.
        adjusted_state (PIL.Image): Transparent image for adjusted robot state.
        landmarks (PIL.Image): Transparent image for detected landmarks.
        probabilities (PIL.Image): White image for storing probability distribution history.
        trail (PIL.Image): Transparent image for storing robot location history.
        arc (float): Width of rays if using arcs to plot measurements.
        points (bool): Use points for measurements.

    """
    def __init__(self, width, height):
        """Initialise map object.

        Args:
            width: Width of grid.
            height: Height of grid.
        """
        self.scanning = False
        self.view_mode = ViewMode.ADJUSTED
        self.probability_mode = ProbabilityMode.PROBABILITIES
        self.origin = np.array([width/2, height/2])
        self.display = Image.new("RGBA", (WIDTH, HEIGHT), (128, 128, 128))
        self.probabilities = False

        self.state_colour = (0, 0, 255)
        self.adjusted_colour = (255, 0, 0)
        self.view_images = {
            ViewMode.ADJUSTED: Image.new("RGBA", (WIDTH, HEIGHT), (128, 128, 128)),
            ViewMode.STATE: Image.new("RGBA", (WIDTH, HEIGHT), (128, 128, 128))
        }

        self.probability_images = {
            ProbabilityMode.PROBABILITIES: Image.new("RGBA", (WIDTH, HEIGHT), (255,255,255)),
            ProbabilityMode.SLAM_PROBABILITIES: Image.new("RGBA", (WIDTH, HEIGHT), (255,255,255)),
            ProbabilityMode.PRIOR_PROBABILITIES: Image.new("RGBA", (WIDTH, HEIGHT), (255,255,255))
        }

    def plot_measurements(self, measurements):
        """Plot a list of measurements.

        Args:
            measurements: List of measurements.
        """
        for measurement in measurements:
            self.plot_measurement(measurement)

    def plot_measurement(self, measurement, colour=(100,100,100)):
        """Plot a measurements.

        Args:
            measurement (Measurement): Measurement to plot.
            colour (tuple): Colour to plot the measurement, in RGBA format.
        """
        draw = ImageDraw.Draw(self.view_images[ViewMode.ADJUSTED])

        location = measurement.state.location + self.origin
        end_location = measurement.location + self.origin

        # Bounding box for measurement
        radius = 2
        measurement_box = ((end_location[0]-radius, end_location[1]-radius),
                     (end_location[0]+radius, end_location[1]+radius))

        draw.line(((location[0], location[1]), (end_location[0], end_location[1])), fill=(200+self.scanning*55,200+self.scanning*55,200+self.scanning*55))
        draw.ellipse(measurement_box, fill=colour)

    def plot_states(self, robot):
        self.plot_state(robot.state, self.state_colour)
        self.plot_state(robot.adjusted, self.adjusted_colour)

    def plot_state(self, state, colour):
        """Plot the location and direction of the robot.

        Args:
            state (State): Robot state.
            colour (tuple): Colour to plot the state.
        """
        draw = ImageDraw.Draw(self.display)
        
        # Calculate the x and y lengths for the robot direction
        end_x = self.origin[0] + state.location[0] + 50 * math.cos(math.radians(state.heading))
        end_y = self.origin[1] + state.location[1] + 50 * math.sin(math.radians(state.heading))

        radius = 3

        # Bounding box for robot
        robot_box = (tuple(state.location + self.origin - radius),
                     tuple(state.location + self.origin + radius))

        # Plot non-adjusted state
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

    def plot_trail(self, state, colour):
        """Plot the current location of the robot as a single pixel.

        Args:
            state (State): Robot state.
            colour (tuple): Colour to plot the state of the robot.
        """
        trail = ImageDraw.Draw(self.display)
        trail.point(tuple((state.location+self.origin).astype(int)), fill=colour)

    def plot_landmarks(self, landmarks, state, colour):
        for landmark in landmarks:
            self.plot_landmark(landmark, state, colour)

    def plot_landmark(self, landmark, state, colour=(0,0,0,255)):
        """Plot a landmark as a triangle, where the two ends of the landmarks are two corners, and the robot is the
        third.

        Args:
            landmark (Landmark): Landmark to plot.
            state (Robot): State of the robot.
            colour (tuple): Colour to plot the landmark
        """
        draw = ImageDraw.Draw(self.display)

        #draw.polygon((tuple(ORIGIN+state.location),tuple((ORIGIN+landmark.segment[0]).astype(int)),
        #              tuple((ORIGIN+landmark.segment[1]).astype(int))), fill=(255,255,255,255))

        draw.line((tuple((self.origin+landmark.segment[0]).astype(int)),
                   tuple((self.origin+landmark.segment[1]).astype(int))), fill=colour)

    def plot_prob_dist(self, dist, dist_type, normalise=True):
        """Given a dictionary mapping coordinates to probability values, plot this distribution, where darker values
        represent more likely locations.

        Args:
            dist (dict): Probability distribution.
            dist_type (ProbabilityMode): Probability mode.
            normalise (bool): Normalise the values so smallest show up as white and largest as black.
        """
        draw = ImageDraw.Draw(self.probability_images[dist_type])
        max_prob = 1
        min_prob = 0

        values = list(dist.values())
        if len(values) > 0 and sum(values) > 0:
            max_prob = max(values)
            if normalise and min(values) != max_prob:
                min_prob = min(values)

        for key in dist:
            value = int(255 - 255 * ((dist[key] - min_prob) / (max_prob - min_prob)))
            draw.point(tuple(self.origin + np.array(key)), fill=(value, value, value))

    def clear(self):
        self.view_images = {
            ViewMode.ADJUSTED: Image.new("RGBA", (WIDTH, HEIGHT), (128, 128, 128)),
            ViewMode.STATE: Image.new("RGBA", (WIDTH, HEIGHT), (128, 128, 128))
        }

    def combine(self, robot, landmarks):
        if self.probabilities:
            return self.probability_images[self.probability_mode]
        else:
            self.display = self.view_images[self.view_mode].copy()
            self.plot_landmarks(landmarks, robot.adjusted, colour=(0,0,0))
            self.plot_states(robot)
            return self.display


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = MapViewer()
    cv.show()
    sys.exit(app.exec_())
