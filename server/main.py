"""Main project module."""

import sys
import threading

import numpy as np
from PIL import ImageQt
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image

# import server.simulation as communication
from server import communication
from server import robot, slam, occupancy
from server.util import TrackingMode, ViewMode, MapMode, SlamMode, LandmarkMode

SPEED = 100


class Main(QtWidgets.QMainWindow):
    """Main class containing robot and for updating it's generated map.
    
    Attributes:
        grid (server.occupancy.Grid): Map object.
        robot (robot.Bot): Robot object
        slam (slam.Slam): Slam object
        communication (communication.Comm): Object for socket communication.
        view_state (ViewState): Contains variables for the current image to display.
    """
    def __init__(self, width, height, error=0, size=None):
        """Initialise MapViewer object.
        
        Args:
            width (int): The width of the map.
            height (int): The height of the map.
        """
        super(Main, self).__init__()

        # Initialise the window and menu items.
        self.imageLabel = QtWidgets.QLabel()
        self.scrollArea = QtWidgets.QScrollArea()
        self.statusBar = QtWidgets.QStatusBar()
        self.initialise_window()
        self.initialise_menu_bar()

        # Setup the robot attributes.
        self.grid = occupancy.Grid(width, height)
        self.robot = robot.Bot()
        self.communication = communication.Comm(self.robot, size=size)
        self.slam = slam.Slam(self.communication, self.grid)
        self.view_state = ViewState(self.slam)

        # Set up image refresh timer.
        timer = QtCore.QTimer(self)
        # noinspection PyUnresolvedReferences
        timer.timeout.connect(self.display)
        timer.start(33)  # 30 Hz

        # Start the robot
        self.communication.start()
        self.slam.start()
        self.view_state.start()

    def wheelEvent(self, event):
        """Detect wheel scroll events, to zoom in/out."""
        degrees = event.angleDelta().y() / 8
        steps = degrees / 15
        self.view_state.scale *= 1.5 ** steps

    def mouseMoveEvent(self, event):
        """Use mouse events to allow dragging of the map to move the view window, when free tracking is enabled."""
        if self.view_state.tracking == TrackingMode.FREE and event.buttons() == QtCore.Qt.LeftButton:
            # Calculate the change in mouse position.
            new_mouse_pos = np.array([event.x(), event.y()])
            mouse_delta = new_mouse_pos - self.view_state.mouse

            # Add this to the view centre.
            self.view_state.centre = self.view_state.centre - mouse_delta * (1 / self.view_state.scale)
            self.view_state.mouse = new_mouse_pos

    def mousePressEvent(self, event):
        """Set the mouse start location, so movement delta can be calculated."""
        if event.buttons() == QtCore.Qt.LeftButton:
            self.view_state.mouse = np.array([event.x(), event.y()])

    def keyPressEvent(self, event):
        """Detect when a key is pressed, and perform the relevant function."""
        key = event.key()  # Get the key.

        # Exit the application.
        if key == QtCore.Qt.Key_Escape:
            self.slam.stop()
            self.communication.stop()
            self.view_state.stop()
            self.close()

        # Remote control commands.
        elif self.slam.allow_control:
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

    def initialise_window(self):
        """Set up the window."""
        self.imageLabel.setBackgroundRole(QtGui.QPalette.Base)
        self.imageLabel.setScaledContents(True)
        self.scrollArea.setWidget(self.imageLabel)
        self.setCentralWidget(self.scrollArea)
        self.scrollArea.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)  # Disable horizontal scrollbar.
        self.scrollArea.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)  # Disable vertical scrollbar.
        self.setWindowTitle("Robot Map")  # Set title.
        self.showFullScreen()  # Make fullscreen.

    # noinspection PyArgumentList,PyUnresolvedReferences
    def initialise_menu_bar(self):
        """Set up the menu bar options."""
        # Set up options to change SLAM type.
        slam_type_menu = self.menuBar().addMenu('SLAM Type')
        slam_type_ag = QtWidgets.QActionGroup(self, exclusive=True)
        slam_type_scan_matching = slam_type_ag.addAction(QtWidgets.QAction('Scan Matching', self, checkable=True))
        slam_type_hough = slam_type_ag.addAction(QtWidgets.QAction('Hough Landmarks', self, checkable=True))
        slam_type_ransac = slam_type_ag.addAction(QtWidgets.QAction('RANSAC Landmarks', self, checkable=True))
        slam_type_scan_matching.triggered.connect(lambda x: self.set_slam_type(SlamMode.SCAN_MATCHING))
        slam_type_hough.triggered.connect(lambda x: self.set_slam_type(LandmarkMode.HOUGH))
        slam_type_ransac.triggered.connect(lambda x: self.set_slam_type(LandmarkMode.RANSAC))
        slam_type_menu.addAction(slam_type_scan_matching)
        slam_type_menu.addAction(slam_type_hough)
        slam_type_menu.addAction(slam_type_ransac)
        self.menuBar().addMenu(slam_type_menu)

        # Set up miscellaneous options.
        options_menu = QtWidgets.QMenu("Options", self)
        options_automatic = QtWidgets.QAction("Automatic", options_menu, checkable=True)
        options_automatic.triggered.connect(self.set_automatic)
        options_menu.addAction(options_automatic)
        self.menuBar().addMenu(options_menu)

        # Set up options for the display.
        display_menu = QtWidgets.QMenu("Display Mode", self)
        display_ag = QtWidgets.QActionGroup(self, exclusive=True)
        display_map = display_ag.addAction(QtWidgets.QAction('Map Distribution', self, checkable=True))
        display_prob = display_ag.addAction(QtWidgets.QAction('Probability Distribution', self, checkable=True))
        display_map.triggered.connect(lambda x: self.set_map_mode(MapMode.DIST))
        display_prob.triggered.connect(lambda x: self.set_map_mode(MapMode.PROB))
        display_menu.addAction(display_map)
        display_menu.addAction(display_prob)
        self.menuBar().addMenu(display_menu)

        # Set up options for how the robot is followed.
        tracking_menu = QtWidgets.QMenu("Tracking Mode", self)
        tracking_ag = QtWidgets.QActionGroup(self, exclusive=True)
        tracking_free = tracking_ag.addAction(QtWidgets.QAction('Free', self, checkable=True))
        tracking_adjusted = tracking_ag.addAction(QtWidgets.QAction('Adjusted Robot', self, checkable=True))
        tracking_raw = tracking_ag.addAction(QtWidgets.QAction('Raw Robot', self, checkable=True))
        tracking_free.triggered.connect(lambda x: self.set_tracking_mode(TrackingMode.FREE))
        tracking_adjusted.triggered.connect(lambda x: self.set_tracking_mode(TrackingMode.ADJUSTED))
        tracking_raw.triggered.connect(lambda x: self.set_tracking_mode(TrackingMode.STATE))
        tracking_menu.addAction(tracking_raw)
        tracking_menu.addAction(tracking_free)
        tracking_menu.addAction(tracking_adjusted)
        self.menuBar().addMenu(tracking_menu)

        # Set up the map display mode.
        map_menu = QtWidgets.QMenu("Map Display Mode", self)
        map_ag = QtWidgets.QActionGroup(self, exclusive=True)
        map_local = map_ag.addAction(QtWidgets.QAction('Local', self, checkable=True))
        map_global = map_ag.addAction(QtWidgets.QAction('Global', self, checkable=True))
        map_local.triggered.connect(lambda x: self.set_display_mode(self.grid.view_mode.LOCAL))
        map_global.triggered.connect(lambda x: self.set_display_mode(self.grid.view_mode.ADJUSTED))
        map_menu.addAction(map_local)
        map_menu.addAction(map_global)
        self.menuBar().addMenu(map_menu)

        # Set up the probability display mode.
        probability_menu = QtWidgets.QMenu("Probability Mode", self)
        probability_ag = QtWidgets.QActionGroup(self, exclusive=True)
        probability_prior = probability_ag.addAction(QtWidgets.QAction('Prior Probabilities', self, checkable=True))
        probability_slam = probability_ag.addAction(QtWidgets.QAction('SLAM Probabilities', self, checkable=True))
        probability_combined = probability_ag.addAction(
            QtWidgets.QAction('Combined Probabilities', self, checkable=True))
        probability_prior.triggered.connect(
            lambda x: self.set_probability_mode(self.grid.probability_mode.PRIOR_PROBABILITIES))
        probability_slam.triggered.connect(
            lambda x: self.set_probability_mode(self.grid.probability_mode.SLAM_PROBABILITIES))
        probability_combined.triggered.connect(
            lambda x: self.set_probability_mode(self.grid.probability_mode.COMBINED_PROBABILITIES))
        probability_menu.addAction(probability_prior)
        probability_menu.addAction(probability_slam)
        probability_menu.addAction(probability_combined)
        self.menuBar().addMenu(probability_menu)

        # Set up the probability slider.
        probability_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        probability_slider.setMinimum(0)
        probability_slider.setMaximum(100)
        probability_slider.setValue(50)
        probability_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        probability_slider.setTickInterval(5)
        probability_slider.valueChanged.connect(self.set_probability_alpha)

        # Add the slider, status bar and title.
        self.statusBar.addWidget(probability_slider)
        self.setWindowTitle("SLAM")
        self.setStatusBar(self.statusBar)

    def display(self):
        """Convert PIL to ImageQt, and display on screen."""
        image_qt = ImageQt.ImageQt(self.view_state.get_image())
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

    def set_display_mode(self, mode):
        """Set the display mode, to switch between viewing probabilities or the map."""
        self.grid.view_mode = mode

    def set_probability_mode(self, mode):
        """Set which probability values to view."""
        self.grid.probability_mode = mode

    def set_map_mode(self, mode):
        """Set the map mode."""
        self.view_state.map_mode = mode

    def set_automatic(self, mode):
        """Set whether the robot moves automatically or not."""
        self.slam.controlled = not mode
        if mode:
            self.slam.resume()

    def set_tracking_mode(self, mode):
        """Set the robot tracking mode."""
        self.view_state.tracking = mode

    def set_probability_alpha(self, value):
        """Set the opacity of the probability overlay."""
        self.view_state.alpha = value / 100

    def set_slam_type(self, mode):
        """Set the SLAM type."""
        if mode == LandmarkMode.RANSAC:
            self.slam.slam_mode = SlamMode.LANDMARKS
            self.slam.landmark_mode = LandmarkMode.RANSAC
        elif mode == LandmarkMode.HOUGH:
            self.slam.slam_mode = SlamMode.LANDMARKS
            self.slam.landmark_mode = LandmarkMode.HOUGH
        else:
            self.slam.slam_mode = SlamMode.SCAN_MATCHING


class ViewState(threading.Thread):
    """Class which keeps track of the image to display.
    
    Attributes:
        alpha (float): The transparency of the probability image to overlay.
        running (bool): True while the tool is open.
        slam (slam.Slam): The main Slam object.
        image (PIL.Image): Image to display.
        scale (float): The zoom factor of the map.
        centre (numpy.ndarray): The coordinates of the central pixel in the display.
        mouse (numpy.ndarray): The coordinates of the mouse.
        dimensions (numpy.ndarray): The dimensions of the screen.
        tracking (TrackingMode): The robot tracking mode.
        map_mode (MapMode): The map mode.
    """
    def __init__(self, sl):
        """Initialise the view state object.
        
        Args:
            sl (slam.Slam): Main SLAM object.
        """
        threading.Thread.__init__(self)
        self.alpha = 0.5
        self.running = True
        self.slam = sl
        self.image = None
        self.scale = 5.0
        self.centre = np.zeros(2)
        self.mouse = np.zeros(2)
        self.dimensions = np.array([1920, 1080])
        self.tracking = TrackingMode.FREE
        self.map_mode = MapMode.DIST

    def run(self):
        """Main thread loop"""
        while self.running:
            """Combine sub-images, and crop based on the scale factor."""
            if self.tracking == TrackingMode.ADJUSTED:
                self.centre = self.slam.comm.robot.adjusted.location
            elif self.tracking == TrackingMode.STATE:
                self.centre = self.slam.comm.robot.state.location

            size = self.dimensions / (self.scale * 2)  # Size of cropped window.
            crop_box = tuple(np.append(self.slam.grid.origin + self.centre - size,
                                       self.slam.grid.origin + self.centre + size))  # Crop box

            self.slam.grid.plot_trail(self.slam.comm.robot.state, ViewMode.STATE)
            self.slam.grid.plot_trail(self.slam.comm.robot.adjusted, ViewMode.ADJUSTED)

            if self.map_mode == MapMode.DIST:
                image = self.slam.grid.combine(self.slam.comm.robot, self.slam.landmarks)
            elif self.map_mode == MapMode.PROB:
                image = Image.blend(self.slam.grid.combine(self.slam.comm.robot, self.slam.landmarks),
                                    self.slam.grid.probability_images[self.slam.grid.probability_mode], self.alpha)
            else:
                image = self.slam.grid.map_images[self.slam.grid.probability_mode]
            self.image = image.crop(crop_box).resize(self.dimensions)  # Update image.

    def stop(self):
        """Stop running."""
        self.running = False

    def get_image(self):
        """Get the current image."""
        return self.image


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = Main(1000, 1000)
    cv.show()
    sys.exit(app.exec_())