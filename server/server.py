import socket
import sys
import math
import threading
import time
import random
from numpy.linalg import norm
from matplotlib import cm
import numpy as np
from scipy import stats
from operator import itemgetter
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt
from scipy.misc import toimage


import scipy.misc
from skimage import draw

# Socket variables
UDP_IP = "10.246.40.233"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

# Map variables
WIDTH = 1920
HEIGHT = 1080
MAP = np.full((HEIGHT,WIDTH), 128, dtype=np.uint8)
#IMAGE = Image.new("L", (WIDTH, HEIGHT), "grey")

RUNNING = True
DIRECTION = 0

HEADING = 0
X = WIDTH/2
Y = HEIGHT/2


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
        #image = ImageQt.ImageQt(IMAGE)
        map = np.copy(MAP)
        plot_state(map)
        image = ImageQt.ImageQt(Image.fromarray(map))
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
            MAP = np.full((HEIGHT,WIDTH), 128, dtype=np.uint8)

        elif key < 128:
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))


def plot_measurement(coords):
    #draw = ImageDraw.Draw(IMAGE)
    #draw.line(, fill=255)
    rr, cc = draw.line(int(coords[1]),int(coords[0]),int(coords[3]),int(coords[2]))
    MAP[rr, cc] =  255
    rr, cc = draw.circle(int(coords[3]),int(coords[2]),3)
    MAP[rr, cc] =  0
    #draw.point([int(coords[i]) for i in range(2,4)], fill=0)


def plot_state(map):
    #draw = ImageDraw.Draw(IMAGE)
    #r = 3
    x_distance = 200 * math.cos(math.radians(HEADING))
    y_distance = 200 * math.sin(math.radians(HEADING))
    #draw.ellipse((x + WIDTH/2-r, y + HEIGHT/2-r, x + WIDTH/2+r, y + HEIGHT/2+r), fill="red")
    #draw.line((x + WIDTH/2, y + HEIGHT/2, x + WIDTH/2 + int(x_distance), y + HEIGHT/2 + int(y_distance)), fill="red")
    #draw.ellipse((x + WIDTH/2 + int(x_distance), y + HEIGHT/2 + int(y_distance)), fill="black")
    rr, cc = draw.line(int(Y+HEIGHT/2),int(X+WIDTH/2),int(Y+HEIGHT/2+y_distance),int(X+WIDTH/2+x_distance))
    map[rr, cc] =  0

    rr, cc = draw.circle(int(Y+HEIGHT/2),int(X+WIDTH/2),5)
    map[rr, cc] =  0


def measurement_to_coords(start_x, start_y, heading, distance=200, sensor=0):
    angle = sensor + heading
    x_distance = distance * math.cos(math.radians(angle))
    y_distance = distance * math.sin(math.radians(angle))
    end_x = start_x + x_distance
    end_y = start_y + y_distance
    return (start_x+WIDTH/2.0, start_y+HEIGHT/2.0, end_x+WIDTH/2.0, end_y+HEIGHT/2.0, angle)


def ransac(N, number_samples, degree_range, max_distance, number_points, points):
    n = 0
    while len(points) > number_points and n < N:
        index = random.randint(0, len(points)-1)
        within_angle = [points[i] for i in range(len(points)) if abs((points[index][4] - points[i][4]) % 360) < degree_range]
        sample = random.sample(within_angle, min(number_samples, len(within_angle)))
        slope, intercept, r_value, p_value, std_err = stats.linregress([sample[i][2] for i in range(len(sample))], [sample[i][3] for i in range(len(sample))])
        

        points_on_line = []
        p1 = np.array([0, intercept])
        p2 = np.array([WIDTH, slope*WIDTH+intercept])
        for point in points:
            p3 = np.array([point[2], point[3]])
            if norm(np.cross(p2-p1, p1-p3))/norm(p2-p1) < max_distance:
                points_on_line.append(point)

        if len(points_on_line) > number_points:
            slope, intercept, r_value, p_value, std_err = stats.linregress([points_on_line[i][2] for i in range(len(points_on_line))], [points_on_line[i][3] for i in range(len(points_on_line))])
            plot_measurement((0, intercept, WIDTH, slope*WIDTH+intercept))
            return (0, intercept, WIDTH, slope*WIDTH+intercept)


class Main(threading.Thread):
    def __init__(self, app):
        threading.Thread.__init__(self)
        self.p_x = 0
        self.p_y = 0


    def run(self):
        #self.ransac_loop()
        while RUNNING:
            self.remote_control_loop()
        SOCK.close()
        app.quit()
        return 0

    def ransac_loop(self):
        SOCK.sendto("C".encode(), (UDP_IP, UDP_PORT))
        measurements = []
        while len(measurements) < 100:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                x, y, heading, angle, forward, backward = [float(x) for x in data.decode().split(',')]
                if forward not in [-1, 255]:
                    measurements.append(measurement_to_coords(x, y, heading, forward, angle/2 + 180))
                if backward not in [-1, 255]:
                    measurements.append(measurement_to_coords(x, y, heading, backward, angle/2))
                #if math.sqrt((x-self.p_x)**2+(y-self.p_y)**2) > 20:
                plot_state(x, y, heading)
                #    self.p_x = x
                #    self.p_y = y
            except BlockingIOError:
                pass
        SOCK.sendto("X".encode(), (UDP_IP, UDP_PORT))
        
        for m in measurements:
            plot_measurement(m)
        ransac(10, 5, 20, 2, 10, measurements)

    def remote_control_loop(self):
        global X, Y, HEADING
        try:
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            X, Y, HEADING, angle, forward, backward = [float(x) for x in data.decode().split(',')]
            if forward not in [-1, 255]:
                plot_measurement(measurement_to_coords(X, Y, HEADING, forward, angle/2 + 180))
            if backward not in [-1, 255]:
                plot_measurement(measurement_to_coords(X, Y, HEADING, backward, angle/2))
            #if math.sqrt((x-self.p_x)**2+(y-self.p_y)**2) > 20:
            #plot_state(x, y, heading)
            #    self.p_x = x
            #    self.p_y = y
        except BlockingIOError:
            pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = Main(app)
    main.start()
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
