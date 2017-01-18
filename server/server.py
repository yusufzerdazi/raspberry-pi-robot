import socket
import sys
import math
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt

# Socket variables
UDP_IP = "10.246.40.233"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

# Probability distributions
DISTANCE_DISTRIBUTION = [0.14, 0.72, 0.14]
ANGLE_DISTRIBUTION = [0.05, 0.1, 0.15, 0.4, 0.15, 0.1, 0.05]

# Map variables
WIDTH = 1920
HEIGHT = 1080
IMAGE = Image.new("L", (WIDTH, HEIGHT), "grey")

RUNNING = True
DIRECTION = 0


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

        elif key < 128:
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))


def plot_measurement(coords):
    draw = ImageDraw.Draw(IMAGE)
    draw.line([int(coords[i]) for i in range(0,4)], fill=255)
    draw.point([int(coords[i]) for i in range(2,4)], fill=0)


def plot_state(x, y, heading):
    draw = ImageDraw.Draw(IMAGE)
    r = 3
    x_distance = 200 * math.cos(math.radians(heading))
    y_distance = 200 * math.sin(math.radians(heading))
    draw.ellipse((x + WIDTH/2-r, y + HEIGHT/2-r, x + WIDTH/2+r, y + HEIGHT/2+r), fill="red")
    draw.line((x + WIDTH/2, y + HEIGHT/2, x + WIDTH/2 + int(x_distance), y + HEIGHT/2 + int(y_distance)), fill="red")


def measurement_to_coords(start_x, start_y, heading, distance=200, sensor=0):
    angle = sensor + heading
    x_distance = distance * math.cos(math.radians(angle))
    y_distance = distance * math.sin(math.radians(angle))
    end_x = start_x + x_distance
    end_y = start_y + y_distance
    return (start_x+WIDTH/2.0, start_y+HEIGHT/2.0, end_x+WIDTH/2.0, end_y+HEIGHT/2.0, angle)


def ransac(points):
    sorted_points = sorted(L, key=itemgetter(4))
    


def run(app):
    p_x = 0
    p_y = 0
    measurements = []
    while RUNNING:
        try:
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            x, y, heading, angle, forward, backward = [float(x) for x in data.decode().split(',')]
            if forward not in [-1, 255]:
                forward_coords = measurement_to_coords(x, y, heading, forward, angle/2 + 180)
                plot_measurement(forward_coords)
            if backward not in [-1, 255]:
                rear_coords = measurement_to_coords(x, y, heading, backward, angle/2)
                plot_measurement(rear_coords)
            if math.sqrt((x-p_x)**2+(y-p_y)**2) > 20:
                plot_state(x, y, heading)
                p_x = x
                p_y = y
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
