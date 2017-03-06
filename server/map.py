import robot
import slam

import sys
import math
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt, ImageChops

WIDTH = 1920
HEIGHT = 1080


class CameraViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraViewer, self).__init__()

        self.map = Map(WIDTH, HEIGHT)
        self.robot = robot.Robot(WIDTH, HEIGHT)
        self.slam = slam.Slam(self.robot, self.map)
        self.slam.start()

        self.scale = 5.0
        self.controlled = True
        self.image = None

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

        self.turning = True
        self.avg = 0
        self.angle = 0
        self.dir = -1

        self.driving = True
        self.avg_dist = 0.0

    def open(self):
        self.crop()
        self.display()

    def crop(self):
        # Set up variables for cropping image
        width = int(WIDTH/(self.scale*2))
        height = int(HEIGHT/(self.scale*2))
        crop_box = ((self.robot.state.location[0]-width, self.robot.state.location[1]-height, self.robot.state.location[0]+width, self.robot.state.location[1]+height))

        self.map.plot_state(self.robot.state)
        self.map.plot_state(self.robot.adjusted(), True)
        self.image = self.map.combine_with_state().crop(crop_box).resize((1920,1080))

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


class Map(object):
    def __init__(self, width, height, points=False):
        # Map variables
        self.image = Image.new("RGBA", (WIDTH, HEIGHT), (128,128,128,255))  # The grey background
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space
        self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.adjusted_state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for robot state
        self.arc = 6

    def plot_measurement(self, measurement):
        if measurement.distance in [-1, 255]:
            return -1
            
        objects = ImageDraw.Draw(self.objects)
        space = ImageDraw.Draw(self.space)

        # Bounding box for arc. Add 1 so that it doesn't intersect the pie.
        arc_box = ((measurement.state.location[0]-(measurement.distance+1), measurement.state.location[1]-(measurement.distance+1)),
                   (measurement.state.location[0]+(measurement.distance+1), measurement.state.location[1]+(measurement.distance+1)))

        # Bounding box for pie.
        pie_box = ((measurement.state.location[0]-measurement.distance, measurement.state.location[1]-measurement.distance),
                   (measurement.state.location[0]+measurement.distance, measurement.state.location[1]+measurement.distance))

        direction = measurement.angle + measurement.state.heading # Absolute robot direction
        start_angle = direction - self.arc # Pie start angle
        end_angle = direction + self.arc # Pie end angle

        objects.arc(arc_box, start_angle, end_angle, fill="black") # Draw arc
        space.pieslice(pie_box, start_angle, end_angle, fill="white") # Draw pie

    def plot_state(self, state, adjusted=False):
        draw = None
        colour = None
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

    def clear(self):
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

    def combine(self):
        return Image.alpha_composite(self.image, Image.alpha_composite(self.space, self.objects))

    def combine_with_state(self):
        return Image.alpha_composite(self.combine(), Image.alpha_composite(self.state, self.adjusted_state))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
