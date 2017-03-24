import sys
import math
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt, ImageChops

import robot_simulator as communication
import robot
import slam
import algebra
import copy


WIDTH = 1920
HEIGHT = 1080
SPEED = 180
ORIGIN = np.array([WIDTH/2, HEIGHT/2])
ORIGIN_STATE = robot.State(ORIGIN)


class MapViewer(QtWidgets.QMainWindow):
    """Main class containing robot and for updating it's generated map.
    
    Attributes:
        map (Map): Map object.
        robot (robot.Robot): Robot object
        slam (slam.Slam): Slam object
        scale (float): Zoom factor for map.
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
        self.scale = 5.0  # Zoom factor.
        self.controlled = controlled  # Remote controlled.
        self.image = None  # Displayed image.
        self.distribution = False  # Whether to display probability distribution or map.

        # Robot attributes.
        self.map = Map(WIDTH, HEIGHT)  # Initialise map.
        self.robot = robot.Robot()  # Initialise robot.

        self.communication = communication.Communication()
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

        # Set up image refresh timer.
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.open)
        timer.start(33)  # 30 Hz

    def open(self):
        """Loop for displaying image."""
        self.update()  # Update robot.
        self.crop()  # Crop and adjust image.
        self.display()  # Display image.

    def update(self):
        observations = self.communication.sense()
        self.robot.update(observations)
        self.map.plot_trail(self.robot.state, (0,0,255,127))
        self.map.plot_trail(self.robot.adjusted, (255,0,0,127))

    def crop(self):
        """Combine sub-images, and crop based on the scale factor."""
        size = np.array([int(WIDTH/(self.scale*2)), int(HEIGHT/(self.scale*2))])  # Size of cropped window.
        crop_box = tuple(np.append(ORIGIN + self.robot.adjusted.location - size,
                                   ORIGIN + self.robot.adjusted.location + size))  # Crop box

        self.map.plot_state(self.robot.state)  # Plot robot from raw data.
        self.map.plot_state(self.robot.adjusted, True)  # Plot SLAM adjusted robot.
        self.image = self.map.combine_with_state(self.distribution).crop(crop_box).resize((1920,1080))  # Update image.

    def display(self):
        """Convert PIL to ImageQt, and display on screen."""
        image_qt = ImageQt.ImageQt(self.image)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

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

        # Zoom out.
        elif key == QtCore.Qt.Key_Equal:
            self.scale = self.scale * 1.5
        # Zoom in.
        elif key == QtCore.Qt.Key_Minus:
            self.scale = self.scale / 1.5

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

            # Toggle between probability distribution and map.
            elif key == QtCore.Qt.Key_P:
                self.distribution = not self.distribution

            # Perform SLAM iteration.
            elif key == QtCore.Qt.Key_Return:  
                self.slam.resume()
                """self.d = True
                self.robot.resume()
                self.robot.sense()
                measurements = []
                while len(measurements) < 1000:
                    sense = self.robot.sense()
                    for new in sense:
                        measurements.append(new)
                        self.map.plot_measurement(new)
                        print(str(new.angle) + ", " + str(new.distance))
                self.d=False
                self.robot.pause()"""


class Map(object):
    """Map class, composed of a number of PIL images, for tracking objects and landmarks.

    Attributes:
        image (PIL.Image): Grey image for background.
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
    def __init__(self, width, height, points=True):
        """Initialise map object.

        Args:
            width: Width of grid.
            height: Height of grid.
            points: Whether to use points or arcs.
        """
        self.image = Image.new("RGBA", (WIDTH, HEIGHT), (128,128,128,255))  # The grey background
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space
        self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.adjusted_state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.landmarks = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for landmarks
        self.probabilities = Image.new("RGBA", (WIDTH, HEIGHT), (255,255,255,255))  # Image for landmarks
        self.trail = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot trail
        self.arc = 6
        self.points = points

    def plot_measurements(self, measurements):
        """Plot a list of measurements.

        Args:
            measurements: List of measurements.
        """
        for measurement in measurements:
            self.plot_measurement(measurement)

    def plot_measurement(self, measurement, colour=(0,0,0,100)):
        """Plot a measurements.

        Args:
            measurement (Measurement): Measurement to plot.
            colour (tuple): Colour to plot the measurement, in RGBA format.
        """
        objects = ImageDraw.Draw(self.objects)
        space = ImageDraw.Draw(self.space)

        location = measurement.state.location + ORIGIN
        end_location = measurement.location + ORIGIN

        # Bounding box for arc. Add 1 so that it doesn't intersect the pie.
        arc_box = ((location[0]-(measurement.distance+1), location[1]-(measurement.distance+1)),
                   (location[0]+(measurement.distance+1), location[1]+(measurement.distance+1)))

        # Bounding box for pie.
        pie_box = ((location[0]-measurement.distance, location[1]-measurement.distance),
                   (location[0]+measurement.distance, location[1]+measurement.distance))

        direction = math.degrees(measurement.angle + measurement.state.heading) # Absolute robot direction
        start_angle = direction - self.arc # Pie start angle
        end_angle = direction + self.arc # Pie end angle

        if not self.points:
            objects.arc(arc_box, start_angle, end_angle, fill="black") # Draw arc
            space.pieslice(pie_box, start_angle, end_angle, fill="white") # Draw pie
        else:
            # Bounding box for measurement
            radius = 2
            measurement_box = ((end_location[0]-radius, end_location[1]-radius),
                         (end_location[0]+radius, end_location[1]+radius))

            space.line(((location[0], location[1]), (end_location[0], end_location[1])), fill=(255,255,255,100))
            space.ellipse(measurement_box, fill=colour)
        

    def plot_state(self, state, adjusted=False):
        """Plot the location and direction of the robot.

        Args:
            state (State): Robot state.
            adjusted (bool): Plotting raw robot or SLAM adjusted robot.
        """
        draw = None
        colour = None
        state += ORIGIN_STATE
        if adjusted:
            self.adjusted_state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0)) # Clear state image
            draw = ImageDraw.Draw(self.adjusted_state)
            colour = (255,0,0,127)
        else:
            self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0)) # Clear state image
            draw = ImageDraw.Draw(self.state)
            colour = (0,0,255,127)
        
        # Calculate the x and y lengths for the robot direction
        end_x = state.location[0] + 50 * math.cos(math.radians(state.heading))
        end_y = state.location[1] + 50 * math.sin(math.radians(state.heading))

        radius = 3

        # Bounding box for robot
        robot_box = ((state.location[0]-radius, state.location[1]-radius),
                     (state.location[0]+radius, state.location[1]+radius))

        # Plot non-adjusted state
        draw.ellipse(robot_box, fill=colour)
        draw.line(((state.location[0], state.location[1]), (end_x, end_y)), fill=colour)

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
        trail = ImageDraw.Draw(self.trail)
        trail.point(tuple((state.location+ORIGIN).astype(int)), fill=colour)

    def plot_landmark(self, landmark, state, colour=(0,0,0,255)):
        """Plot a landmark as a triangle, where the two ends of the landmarks are two corners, and the robot is the
        third.

        Args:
            landmark (Landmark): Landmark to plot.
            state (Robot): State of the robot.
        """
        draw = ImageDraw.Draw(self.landmarks)

        draw.polygon((tuple(ORIGIN+state.location),tuple((ORIGIN+landmark.start).astype(int)),
                      tuple((ORIGIN+landmark.end).astype(int))), fill=(255,255,255,255))

        draw.line((tuple((ORIGIN+landmark.start).astype(int)),
                   tuple((ORIGIN+landmark.end).astype(int))), fill=colour)

    def plot_prob_dist(self, dist):
        """Given a dictionary mapping coordinates to probability values, plot this distribution, where darker values
        represent more likely locations.

        Args:
            dist (dict): Probability distribution.
        """
        draw = ImageDraw.Draw(self.probabilities)
        for key in dist:
            draw.point((ORIGIN[0]+key[0], ORIGIN[1]+key[1]), fill=(int(255-255*dist[key]), 
                                                                   int(255-255*dist[key]), 
                                                                   int(255-255*dist[key]), 255))

    def clear(self):
        """Clears objects and space."""
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

    def combine(self):
        """Returns composite of above images."""
        return Image.alpha_composite(self.image, Image.alpha_composite(self.space, 
            Image.alpha_composite(self.objects, self.landmarks)))

    def combine_with_state(self, dist):
        """Returns above composite with robot state overlayed on top.

        Args:
            dist (bool): Display probability distribution instead of map."""
        if dist:
            return self.probabilities
        return Image.alpha_composite(self.combine(), Image.alpha_composite(self.state,
            Image.alpha_composite(self.adjusted_state, self.trail)))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = MapViewer()
    cv.show()
    sys.exit(app.exec_())
