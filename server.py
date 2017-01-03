import socket
import sys
import math
import threading
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt

# Socket variables
UDP_IP = "192.168.0.41"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

# Robot constants
ROBOT_WIDTH = 14 # Distance between wheels (cm)
CM_PER_DEGREE = 36.1/720.0

# Map variables
WIDTH = 1920
HEIGHT = 1080
IMAGE = Image.new("L", (WIDTH, HEIGHT), "grey")

# Robot variables
X = float(WIDTH/2)
Y = float(HEIGHT/2)
HEADING = 90

# Sensor values
RIGHT_WHEEL = 0
LEFT_WHEEL = 0
ANGLE = 0
FORWARD = 0
BACKWARD = 0

RUNNING = True


class CameraViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraViewer, self).__init__()

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
        # Get data and display
        image = ImageQt.ImageQt(IMAGE)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image))
        self.imageLabel.adjustSize()

    def keyPressEvent(self, event):
        global RUNNING

        # If escape pressed, quit the app
        if event.key() == QtCore.Qt.Key_Escape:
            RUNNING = False
            self.close()

        # If the key is an ASCII character, send it to the robot
        c = event.key()
        if c < 128:
            SOCK.sendto(chr(c).encode(), (UDP_IP, UDP_PORT))


def calculate_new_position(left_delta, right_delta):
    global X, Y, HEADING

    # Convert the deltas to centimetres
    left_delta_cm = CM_PER_DEGREE*left_delta*2.0
    right_delta_cm = CM_PER_DEGREE*right_delta*2.0

    # If moved in straight line
    if left_delta_cm == right_delta_cm:
        X = X + left_delta_cm * math.cos(math.radians(HEADING))
        Y = Y + right_delta_cm * math.sin(math.radians(HEADING))
    else:
        R = ROBOT_WIDTH * (left_delta_cm + right_delta_cm) / (2 * (right_delta_cm - left_delta_cm))
        wd = (right_delta_cm - left_delta_cm) / ROBOT_WIDTH

        X = X + R * math.sin(math.radians(wd + HEADING)) - R * math.sin(math.radians(HEADING))
        Y = Y - R * math.cos(math.radians(wd + HEADING)) + R * math.cos(math.radians(HEADING))

        HEADING = (HEADING + wd) % 360


def update_map(distance, angle):
    if distance not in [-1, 255]:
        angle = (angle/2)%360
        draw = ImageDraw.Draw(IMAGE)

        x_distance = distance * math.cos(math.radians(angle))
        y_distance = distance * math.sin(math.radians(angle))

        draw.line((X, Y, X + int(x_distance), Y + int(y_distance)), fill=255)
        draw.point((X + int(x_distance), Y + int(y_distance)), fill=0)


def run(app):
    global RIGHT_WHEEL, LEFT_WHEEL, ANGLE, FORWARD, BACKWARD, RUNNING
    while RUNNING:
        try:
            draw = ImageDraw.Draw(IMAGE)
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            right_wheel, left_wheel, angle, forward, backward = [float(x) for x in data.decode().split(',')]
            calculate_new_position(left_wheel-LEFT_WHEEL, right_wheel-RIGHT_WHEEL)
            update_map(forward, angle)
            update_map(backward, angle+180)
            RIGHT_WHEEL, LEFT_WHEEL, ANGLE, FORWARD, BACKWARD = right_wheel, left_wheel, angle, forward, backward
        except BlockingIOError:
            pass
    SOCK.close()
    app.quit()
    return 0


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    threading.Thread(target=run, args=[app]).start()

    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
