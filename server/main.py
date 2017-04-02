import sys
import threading

import numpy as np
from PIL import ImageQt
from PyQt5 import QtCore, QtGui, QtWidgets

import server.robot_simulator as communication
from server import model
from server import communication
from server import robot, slam, view
from server.util import TrackingMode, ViewMode, ProbabilityMode


SPEED=120


class Main(QtWidgets.QMainWindow):
    """Main class containing robot and for updating it's generated map.
    
    Attributes:
        map (server.view.Occupancy): Map object.
        robot (robot.Robot): Robot object
        slam (slam.Slam): Slam object
        view_scale (float): Zoom factor for map.
        controlled (bool): Keyboard or automatic robot movement.
        image (PIL.Image): Image to display on screen.
        distribution (PIL.Image): Probability distribution history from SLAM.
    """
    def __init__(self, width, height, controlled=True):
        """Initialise MapViewer object.

        Args:
            controlled: Whether the robot is controlled via input or automatically.
        """
        super(Main, self).__init__()

        # Map attributes

        self.controlled = controlled  # Remote controlled.
        self.distribution = False  # Whether to display probability distribution or map.
        self.running = True

        # Robot attributes.
        self.map = view.Occupancy(width, height)  # Initialise map.
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

        self.cropthread = threading.Thread(target=self.crop).start()

    def update(self):
        observations = self.communication.sense()
        self.robot.update(observations)
        self.map.plot_trail(self.robot.state, (0,0,255,127))
        self.map.plot_trail(self.robot.adjusted, (255,0,0,127))

    def crop(self):
        while self.running:
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
            self.slam.stop()
            self.slam.resume()
            self.communication.stop()
            self.running = False
            self.close()
        # Clear the screen.
        elif key == QtCore.Qt.Key_Q:
            self.map.clear()
        elif key == QtCore.Qt.Key_1:
            self.tracking_mode = TrackingMode.FREE
        elif key == QtCore.Qt.Key_2:
            self.tracking_mode = TrackingMode.ADJUSTED
        elif key == QtCore.Qt.Key_3:
            self.tracking_mode = TrackingMode.STATE

        elif key == QtCore.Qt.Key_5:
            self.map.view_mode = ViewMode.ADJUSTED
        elif key == QtCore.Qt.Key_6:
            self.map.view_mode = ViewMode.LOCAL

        elif key == QtCore.Qt.Key_4:
            self.map.probability_mode = ProbabilityMode.LOCAL_MAP
        elif key == QtCore.Qt.Key_7:
            self.map.probability_mode = ProbabilityMode.GLOBAL_MAP
        elif key == QtCore.Qt.Key_8:
            self.map.probability_mode = ProbabilityMode.SLAM_PROBABILITIES
        elif key == QtCore.Qt.Key_9:
            self.map.probability_mode = ProbabilityMode.PRIOR_PROBABILITIES
        elif key == QtCore.Qt.Key_0:
            self.map.probability_mode = ProbabilityMode.COMBINED_PROBABILITIES

        # Toggle between probability distribution and map.
        elif key == QtCore.Qt.Key_P:
            self.map.probabilities = not self.map.probabilities

        # Toggle between probability distribution and map.
        elif key == QtCore.Qt.Key_L:
            self.slam.calibrate()

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


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = Main(1000, 1000)
    cv.show()
    sys.exit(app.exec_())
