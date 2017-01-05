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
HEADING = 0

# Sensor values
RIGHT_WHEEL = 0
LEFT_WHEEL = 0
ANGLE = 0
FORWARD = 0
BACKWARD = 0

RUNNING = True
TURNING = 0


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
        global RUNNING, RIGHT_WHEEL, LEFT_WHEEL, TURNING
        key = event.key()

        # If escape pressed, quit the app
        if key == QtCore.Qt.Key_Escape:
            SOCK.sendto("STOP".encode(), (UDP_IP, UDP_PORT))
            RUNNING = False
            self.close()

        elif key == QtCore.Qt.Key_Q:
            draw = ImageDraw.Draw(IMAGE)
            draw.rectangle([(0,0),(1919,1079)], fill="grey", outline=None)

        elif key == QtCore.Qt.Key_D or key == QtCore.Qt.Key_A:
            if key == QtCore.Qt.Key_D:
                TURNING = 1
            else:
                TURNING = -1
            right_wheel = RIGHT_WHEEL
            left_wheel = LEFT_WHEEL
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))
            while abs(right_wheel-RIGHT_WHEEL) < 200 and abs(left_wheel-LEFT_WHEEL) < 200:
                pass
            SOCK.sendto("Z".encode(), (UDP_IP, UDP_PORT))
            TURNING = 0

        elif key < 128:
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))


def calculate_new_position(left_delta, right_delta):
    global X, Y, HEADING

    # Convert the deltas to centimetres
    left_distance = CM_PER_DEGREE*left_delta*2.0
    right_distance = CM_PER_DEGREE*right_delta*2.0
    abs_distance = (abs(left_distance)+abs(right_distance))/2
    avg_distance = (left_distance+right_distance)/2
    turning = TURNING

    if turning:
        HEADING = HEADING + turning*360/(math.pi*ROBOT_WIDTH)
    else:
        X = X + avg_distance * math.cos(math.radians(HEADING))
        Y = Y + avg_distance * math.sin(math.radians(HEADING))


def update_map(distance, angle, color=255):
    if distance not in [-1, 255]:
        draw = ImageDraw.Draw(IMAGE)

        x_distance = distance * math.cos(math.radians(angle))
        y_distance = distance * math.sin(math.radians(angle))

        draw.line((X, Y, X + int(x_distance), Y + int(y_distance)), fill=color)
        draw.point((X + int(x_distance), Y + int(y_distance)), fill=0)


def run(app):
    global RIGHT_WHEEL, LEFT_WHEEL, ANGLE, FORWARD, BACKWARD, RUNNING
    while RUNNING:
        try:
            draw = ImageDraw.Draw(IMAGE)
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            right_wheel, left_wheel, angle, forward, backward = [float(x) for x in data.decode().split(',')]
            calculate_new_position(left_wheel-LEFT_WHEEL, right_wheel-RIGHT_WHEEL)
            update_map(forward, angle/2 + 180)
            update_map(backward, angle/2)
            update_map(250, HEADING, "red")
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
