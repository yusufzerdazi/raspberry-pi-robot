import sys
import threading

import numpy as np
from PIL import ImageQt
from PyQt5 import QtCore, QtGui, QtWidgets

import server.robot_simulator as communication
#from server import communication
from server import robot, slam, view
from server.util import TrackingMode, ViewMode, MapMode

SPEED=120


class Main(QtWidgets.QMainWindow):
    """Main class containing robot and for updating it's generated map.
    
    Attributes:
        grid (server.view.Grid): Map object.
        robot (robot.Bot): Robot object
        slam (slam.Slam): Slam object
    """
    def __init__(self, width, height):
        """Initialise MapViewer object.

        Args:
            controlled: Whether the robot is controlled via input or automatically.
        """
        super(Main, self).__init__()

        # Initialise the window and menu items.
        self.initialise_window()
        self.initialise_menus()

        # Setup the robot attributes.
        self.grid = view.Grid(width, height)
        self.robot = robot.Bot()
        self.communication = communication.Comm(self.robot)
        self.slam = slam.Slam(self.communication, self.grid)
        self.refresh = Update(self.slam)

        # Set up image refresh timer.
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.display)
        timer.start(33)  # 30 Hz

        # Start the robot
        self.communication.start()
        self.slam.start()
        self.refresh.start()

    def initialise_window(self):
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

    def initialise_menus(self):
        menu = QtWidgets.QMenu("SLAM", self)
        ag = QtWidgets.QActionGroup(self, exclusive=True)
        a = ag.addAction(QtWidgets.QAction('Scan Matching', self, checkable=True))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Landmarks', self, checkable=True))
        menu.addAction(a)
        self.menuBar().addMenu(menu)

        menu = QtWidgets.QMenu("Options", self)
        a = QtWidgets.QAction("Show Probabilities", menu, checkable=True)
        a.triggered.connect(self.switch_view_mode)
        menu.addAction(a)
        self.menuBar().addMenu(menu)
        a = QtWidgets.QAction("Automatic", menu, checkable=True)
        a.triggered.connect(self.switch_control)
        menu.addAction(a)
        self.menuBar().addMenu(menu)

        menu = QtWidgets.QMenu("Display Mode", self)
        ag = QtWidgets.QActionGroup(self, exclusive=True)
        a = ag.addAction(QtWidgets.QAction('Map Distribution', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_map_mode(MapMode.DIST))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Probability Distribution', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_map_mode(MapMode.PROB))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Map', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_map_mode(MapMode.FINAL))
        menu.addAction(a)
        self.menuBar().addMenu(menu)

        menu = QtWidgets.QMenu("Tracking Mode", self)
        ag = QtWidgets.QActionGroup(self, exclusive=True)
        a = ag.addAction(QtWidgets.QAction('Free', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_tracking_mode(TrackingMode.FREE))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Adjusted Robot', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_tracking_mode(TrackingMode.ADJUSTED))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Raw Robot', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_tracking_mode(TrackingMode.STATE))
        menu.addAction(a)
        self.menuBar().addMenu(menu)

        menu = QtWidgets.QMenu("Map Display Mode", self)
        ag = QtWidgets.QActionGroup(self, exclusive=True)
        a = ag.addAction(QtWidgets.QAction('Local', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_display_mode(self.grid.view_mode.LOCAL))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Global', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_display_mode(self.grid.view_mode.ADJUSTED))
        menu.addAction(a)
        self.menuBar().addMenu(menu)

        menu = QtWidgets.QMenu("Probability Mode", self)
        ag = QtWidgets.QActionGroup(self, exclusive=True)
        a = ag.addAction(QtWidgets.QAction('Prior Probabilities', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_probability_mode(self.grid.probability_mode.PRIOR_PROBABILITIES))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('SLAM Probabilities', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_probability_mode(self.grid.probability_mode.SLAM_PROBABILITIES))
        menu.addAction(a)
        a = ag.addAction(QtWidgets.QAction('Combined Probabilities', self, checkable=True))
        a.triggered.connect(lambda x: self.switch_probability_mode(self.grid.probability_mode.COMBINED_PROBABILITIES))
        menu.addAction(a)
        self.menuBar().addMenu(menu)

    def switch_view_mode(self, prob):
        self.grid.probabilities = prob

    def switch_display_mode(self, mode):
        self.grid.view_mode = mode

    def switch_probability_mode(self, mode):
        self.grid.probability_mode = mode

    def switch_tracking_mode(self, mode):
        self.refresh.tracking = mode

    def switch_control(self, mode):
        self.slam.controlled = not mode
        if mode:
            self.slam.resume()

    def switch_map_mode(self, mode):
        self.refresh.map_mode = mode

    def display(self):
        """Convert PIL to ImageQt, and display on screen."""
        image_qt = ImageQt.ImageQt(self.refresh.get_image())
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

    def wheelEvent(self, event):
        """Zoom in/out"""
        degrees = event.angleDelta().y() / 8
        steps = degrees / 15
        self.refresh.scale *= 1.5 ** steps

    def mouseMoveEvent(self, event):
        """Allow dragging of the map to move the view window, when free tracking is enabled."""
        if self.refresh.tracking == TrackingMode.FREE and event.buttons() == QtCore.Qt.LeftButton:

            # Calculate the change in mouse position.
            new_mouse_pos = np.array([event.x(), event.y()])
            mouse_delta = new_mouse_pos - self.refresh.mouse

            # Add this to the view centre.
            self.refresh.centre = self.refresh.centre - mouse_delta * (1 / self.refresh.scale)
            self.refresh.mouse = new_mouse_pos

    def mousePressEvent(self, event):
        """Set the mouse start location, so movement delta can be calculated."""
        if event.buttons() == QtCore.Qt.LeftButton:
            self.refresh.mouse = np.array([event.x(), event.y()])

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
            self.refresh.stop()
            self.close()

        # Remote control commands.
        elif self.slam.controlled:
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


class Update(threading.Thread):
    def __init__(self, sl):
        threading.Thread.__init__(self)
        self.running = True
        self.slam = sl
        self.image = None
        self.scale = 5.0
        self.slam = sl
        self.centre = np.zeros(2)
        self.mouse = np.zeros(2)
        self.dimensions = np.array([1920, 1080])
        self.tracking = TrackingMode.FREE
        self.map_mode = MapMode.DIST

    def run(self):
        while self.running:
            """Combine sub-images, and crop based on the scale factor."""
            if self.tracking == TrackingMode.ADJUSTED:
                self.centre = self.slam.comm.robot.adjusted.location
            elif self.tracking == TrackingMode.STATE:
                self.centre = self.slam.comm.robot.state.location

            l = self.slam.comm.get_measurements()
            for measurement in l:
                self.slam.grid.plot_measurement(measurement)

            size = self.dimensions / (self.scale * 2)  # Size of cropped window.
            crop_box = tuple(np.append(self.slam.grid.origin + self.centre - size,
                                       self.slam.grid.origin + self.centre + size))  # Crop box
            image = self.slam.grid.combine(self.slam.comm.robot, self.slam.landmarks)
            self.slam.grid.plot_trail(self.slam.comm.robot.state, ViewMode.STATE)
            self.slam.grid.plot_trail(self.slam.comm.robot.adjusted, ViewMode.ADJUSTED)

            if self.map_mode == MapMode.PROB:
                self.image = self.slam.grid.probability_images[self.slam.grid.probability_mode]
            elif self.map_mode == MapMode.FINAL:
                self.image = self.slam.grid.map_images[self.slam.grid.probability_mode]
            else:
                self.image = image.crop(crop_box).resize(self.dimensions)  # Update image.

    def stop(self):
        self.running = False

    def get_image(self):
        return self.image


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = Main(1000, 1000)
    cv.show()
    sys.exit(app.exec_())
