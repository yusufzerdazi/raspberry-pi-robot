import robot#_simulator as robot
import slam
import algebra

import sys
import math
import time

import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt, ImageChops

WIDTH = 1920
HEIGHT = 1080
ORIGIN = np.array([WIDTH/2, HEIGHT/2])
ORIGIN_STATE = robot.State(ORIGIN)


class CameraViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraViewer, self).__init__()

        self.map = Map(WIDTH, HEIGHT)
        self.robot = robot.Robot()
        self.slam = slam.Slam(self.robot, self.map)
        self.slam.start()
        self.robot.resume()

        self.scale = 5.0
        self.controlled = True
        self.image = None
        self.dist = False

        # Set up the window
        self.imageLabel = QtWidgets.QLabel()
        self.imageLabel.setBackgroundRole(QtGui.QPalette.Base)
        self.imageLabel.setScaledContents(True)

        self.scrollArea = QtWidgets.QScrollArea()
        self.scrollArea.setWidget(self.imageLabel)
        self.setCentralWidget(self.scrollArea)
        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        self.setWindowTitle("Map")
        self.showFullScreen()

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.open)
        timer.start(33)  # 30 Hz

    def open(self):
        self.crop()
        self.display()

    def crop(self):
        # Set up variables for cropping image
        size = np.array([int(WIDTH/(self.scale*2)), int(HEIGHT/(self.scale*2))])

        crop_box = tuple(np.append(ORIGIN + self.robot.adjusted.location - size, ORIGIN + self.robot.adjusted.location + size))

        self.map.plot_state(self.robot.state)
        self.map.plot_state(self.robot.adjusted, True)
        self.image = self.map.combine_with_state(self.dist).crop(crop_box).resize((1920,1080))

    def display(self):
        image_qt = ImageQt.ImageQt(self.image)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

    def keyPressEvent(self, event):
        key = event.key()

        if key == QtCore.Qt.Key_Escape:
            self.close()
            self.slam.stop()
            self.robot.stop()
            
        elif key == QtCore.Qt.Key_Q:
            self.map.clear()

        elif key == QtCore.Qt.Key_Equal:
            self.scale = self.scale * 1.5

        elif key == QtCore.Qt.Key_Minus:
            self.scale = self.scale / 1.5

        elif self.controlled:
            if key == QtCore.Qt.Key_W:
                self.robot.move(255, False)

            elif key == QtCore.Qt.Key_A:
                self.robot.move(-255, True)

            elif key == QtCore.Qt.Key_S:
                self.robot.move(-255, False)

            elif key == QtCore.Qt.Key_D:
                self.robot.move(255, True)

            elif key == QtCore.Qt.Key_Z:
                self.robot.move(0, False)

            elif key == QtCore.Qt.Key_X:
                self.robot.pause()

            elif key == QtCore.Qt.Key_C:
                self.robot.resume()

            elif key == QtCore.Qt.Key_P:
                self.dist = not self.dist


class Map(object):
    def __init__(self, width, height, points=False):
        # Map variables
        self.image = Image.new("RGBA", (WIDTH, HEIGHT), (128,128,128,255))  # The grey background
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space
        self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.adjusted_state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.landmarks = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for landmarks
        self.probabilities = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,255))  # Image for landmarks
        self.arc = 6

    def plot_measurements(self, measurements):
        for measurement in measurements:
            self.plot_measurement(measurement)

    def plot_measurement(self, measurement):
        if measurement.distance in [-1, 255]:
            return -1
            
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

        direction = measurement.angle + measurement.state.heading # Absolute robot direction
        start_angle = direction - self.arc # Pie start angle
        end_angle = direction + self.arc # Pie end angle

        #objects.arc(arc_box, start_angle, end_angle, fill="black") # Draw arc
        #space.pieslice(pie_box, start_angle, end_angle, fill="white") # Draw pie

        # Bounding box for measurement
        radius = 2
        measurement_box = ((end_location[0]-radius, end_location[1]-radius),
                     (end_location[0]+radius, end_location[1]+radius))

        space.line(((location[0], location[1]), (end_location[0], end_location[1])), fill=(255,255,255,100))
        space.ellipse(measurement_box, fill=(0,0,0,100))
        

    def plot_state(self, state, adjusted=False):
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
        cw_x = end_x + 10 * math.cos(math.radians(state.heading+135))
        cw_y = end_y + 10 * math.sin(math.radians(state.heading+135))

        # Counterclockwise part of arrow head
        ccw_x = end_x + 10 * math.cos(math.radians(state.heading-135))
        ccw_y = end_y + 10 * math.sin(math.radians(state.heading-135))

        # Plot arrow head
        draw.line((end_x, end_y, cw_x, cw_y), fill=colour)
        draw.line((end_x, end_y, ccw_x, ccw_y), fill=colour)

    def plot_landmark(self, landmark, robot, colour=(255,0,0,255)):
        #self.plot_line(landmark.a, landmark.b, colour)
        draw = ImageDraw.Draw(self.landmarks)
        #draw.polygon((tuple(ORIGIN+robot.location),tuple((ORIGIN+landmark.start).astype(int)),tuple((ORIGIN+landmark.end).astype(int))), fill=(255,255,255,255))
        draw.line((tuple((ORIGIN+landmark.start).astype(int)),tuple((ORIGIN+landmark.end).astype(int))), fill=(0,0,0,255))

    def plot_line(self, a, b, colour):
        draw = ImageDraw.Draw(self.landmarks)
        A = algebra.line_intersection(a, b, 0, ORIGIN[1])
        if A[1] < -ORIGIN[0]:
            A = np.array([-ORIGIN[0], b-a*ORIGIN[0]])

        B = algebra.line_intersection(a, b, 0, -ORIGIN[1])
        if B[1] > ORIGIN[0]:
            B = np.array([ORIGIN[0], b+a*ORIGIN[0]])

        draw.line((tuple(A+ORIGIN), tuple(B+ORIGIN)), fill=colour)

    def plot_prob_dist(self, dist):
        #self.probabilities = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,255))
        draw = ImageDraw.Draw(self.probabilities)
        for key in dist:
            draw.point((ORIGIN[0]+key[0], ORIGIN[1]+key[1]), fill=(int(255*dist[key]), int(255*dist[key]), int(255*dist[key]), 255))

    def clear(self):
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

    def combine(self):
        return Image.alpha_composite(self.image, Image.alpha_composite(self.space, Image.alpha_composite(self.objects, self.landmarks)))

    def combine_with_state(self, dist):
        if dist:
            return self.probabilities
        return Image.alpha_composite(self.combine(), Image.alpha_composite(self.state, self.adjusted_state))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
