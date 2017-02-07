import socket
import sys
import math
import threading
import time
import random

import numpy as np
from scipy import stats
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt

# Socket variables
UDP_IP = "10.246.40.233"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

# Map variables
WIDTH = 1920  # Image width
HEIGHT = 1080  # Image height
IMAGE = Image.new("RGBA", (WIDTH, HEIGHT), (128,128,128,255))  # The grey background
OBJECTS = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
SPACE = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

# Robot variables
RUNNING = True  # Robot is running
HEADING = 0  # Robot starts facing right
COORDS = np.array([0.0,0.0])

ADJUSTMENT = np.array([0.0,0.0])


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
        # Overlay space, then objects, then robot position.
        image = Image.alpha_composite(IMAGE, SPACE)
        image = Image.alpha_composite(image, OBJECTS)
        plot_state(image)

        # Convert to greyscale and display
        display = image.convert("L")
        image_qt = ImageQt.ImageQt(display)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image_qt))
        self.imageLabel.adjustSize()

    def keyPressEvent(self, event):
        global RUNNING
        key = event.key()

        if key == QtCore.Qt.Key_Escape:
            # If escape pressed, quit the app
            SOCK.sendto("STOP".encode(), (UDP_IP, UDP_PORT))
            RUNNING = False
            self.close()

        elif key == QtCore.Qt.Key_Q:
            # Clear the image
            OBJECTS = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))
            SPACE = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))

        elif key < 128:
            # Send ASCII key to robot.
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))


def plot_measurement(x, y, heading, distance, angle, mode="arc", point_radius=12, arc_angle=12):
    objects_draw = ImageDraw.Draw(OBJECTS)
    space_draw = ImageDraw.Draw(SPACE)

    x_end = x+distance*math.cos(math.radians(angle+heading))
    y_end = y+distance*math.sin(math.radians(angle+heading))

    if mode == "point":
        arc_angle = math.degrees(math.asin(point_radius/distance))

    if mode == "arc":
        objects_draw.arc([x + ADJUSTMENT[0]+WIDTH/2-distance-1, y + ADJUSTMENT[1]+HEIGHT/2-distance-1, x + ADJUSTMENT[0]+WIDTH/2+distance+1, y + ADJUSTMENT[1]+HEIGHT/2+distance+1], heading+angle-arc_angle, heading+angle+arc_angle, fill=(0,0,0,255))
    elif mode == "point":
        objects_draw.ellipse([x_end + ADJUSTMENT[0]+WIDTH/2-point_radius, y_end + ADJUSTMENT[1]+HEIGHT/2-point_radius, x_end + ADJUSTMENT[0]+WIDTH/2+point_radius, y_end + ADJUSTMENT[1]+HEIGHT/2+point_radius], fill=(0,0,0,255))

    if arc_angle == 1:
        space_draw.line([x + ADJUSTMENT[0]+WIDTH/2, y + ADJUSTMENT[1]+HEIGHT/2, x_end + ADJUSTMENT[0]+WIDTH/2, y_end + ADJUSTMENT[1]+HEIGHT/2], fill=(255,255,255,255))
    else:
        space_draw.pieslice([x + ADJUSTMENT[0]+WIDTH/2-distance, y + ADJUSTMENT[1]+HEIGHT/2-distance, x + ADJUSTMENT[0]+WIDTH/2+distance, y + ADJUSTMENT[1]+HEIGHT/2+distance], heading+angle-arc_angle, heading+angle+arc_angle, fill=(255,255,255,255))


def plot_state(image):
    draw = ImageDraw.Draw(image)
    r = 3
    x_distance = 50 * math.cos(math.radians(HEADING))
    y_distance = 50 * math.sin(math.radians(HEADING))
    # Plot non-adjusted state
    draw.ellipse((COORDS[0] + WIDTH/2-r, COORDS[1] + HEIGHT/2-r, COORDS[0] + WIDTH/2+r, COORDS[1] + HEIGHT/2+r), fill=(100,0,0,255))
    draw.line((COORDS[0] + WIDTH/2, COORDS[1] + HEIGHT/2, COORDS[0] + WIDTH/2 + int(x_distance), COORDS[1] + HEIGHT/2 + int(y_distance)), fill=(255,0,0,255))

    # Plot adjusted state
    draw.ellipse((COORDS[0] + ADJUSTMENT[0] + WIDTH/2-r, COORDS[1] + ADJUSTMENT[1] + HEIGHT/2-r, COORDS[0] + ADJUSTMENT[0] + WIDTH/2+r, COORDS[1] + ADJUSTMENT[1] + HEIGHT/2+r), fill=(255,0,0,255))
    draw.line((COORDS[0] + ADJUSTMENT[0] + WIDTH/2, COORDS[1] + ADJUSTMENT[1] + HEIGHT/2, COORDS[0] + ADJUSTMENT[0] + WIDTH/2 + int(x_distance), COORDS[1] + ADJUSTMENT[1] + HEIGHT/2 + int(y_distance)), fill=(255,0,0,255))


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
            if np.linalg.norm(np.cross(p2-p1, p1-p3))/norm(p2-p1) < max_distance:
                points_on_line.append(point)

        if len(points_on_line) > number_points:
            slope, intercept, r_value, p_value, std_err = stats.linregress([points_on_line[i][2] for i in range(len(points_on_line))], [points_on_line[i][3] for i in range(len(points_on_line))])
            plot_measurement((0, intercept, WIDTH, slope*WIDTH+intercept))
            return (0, intercept, WIDTH, slope*WIDTH+intercept)


class Main(threading.Thread):
    def __init__(self, app):
        threading.Thread.__init__(self)

    def run(self):
        global ADJUSTMENT
        #self.ransac_loop()
        while RUNNING:
            self.continuous_measurement()
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
                #if forward not in [-1, 255]:
                measurements.append(measurement_to_coords(x, y, heading, forward, angle/2 + 180))
                #if backward not in [-1, 255]:
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

    def incremental_measurement(self):
        global COORDS, HEADING
        meas = []
        while True:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                x, y, heading, angle, forward, backward = [float(x) for x in data.decode().split(',')]
                if x == y == heading == angle == forward == backward == -1.0:
                    break
                else:
                    COORDS[0] = x
                    COORDS[1] = y
                    HEADING = heading
                if forward not in [-1, 255]:
                    meas.append([COORDS[0], COORDS[1], HEADING, forward, (angle/2 + 180)%360])
                if backward not in [-1, 255]:
                    meas.append([COORDS[0], COORDS[1], HEADING, backward, (angle/2)%360])

            except BlockingIOError:
                pass

        deg = 12
        while deg <= 348:
            s = [m for m in meas if deg-24 <= m[4] < deg]
            if len(s) > 3:
                min_element = s[0]
                for el in s:
                    if el[3] < min_element[3]:
                        min_element = el
                plot_measurement(*min_element[:4], deg)
            deg += 24
        meas = []

    def continuous_measurement(self):
        global COORDS, HEADING
        meas = []
        try:
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            x, y, heading, angle, forward, backward = [float(x) for x in data.decode().split(',')]
            if x == y == heading == angle == forward == backward == -1.0:
                pass
            else:
                COORDS[0] = x
                COORDS[1] = y
                HEADING = heading
                if forward not in [-1, 255]:
                    plot_measurement(COORDS[0], COORDS[1], HEADING, forward, angle/2 + 180, "point", 3)
                if backward not in [-1, 255]:
                    plot_measurement(COORDS[0], COORDS[1], HEADING, backward, angle/2, "point", 3)
        except BlockingIOError:
            pass

    def slam(self):
        global ADJUSTMENT, COORDS, RUNNING, HEADING
        meas = []
        O = np.copy(COORDS)
        while len(meas) < 100 and RUNNING:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                COORDS[0], COORDS[1], HEADING, angle, forward, backward = [float(x) for x in data.decode().split(',')]
                if backward not in [-1, 255]:
                    plot_measurement(COORDS[0], COORDS[1], HEADING, backward, angle/2, "point", 3)
                    meas.append(backward)
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))
        mean_initial_dist = np.min(np.array(meas))

        print("Initial distance = " + str(mean_initial_dist))

        
        SOCK.sendto("W".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.5)
        SOCK.sendto("Z".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.5)

        meas2=[]
        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        while len(meas2) < 100 and RUNNING:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                COORDS[0], COORDS[1], HEADING, angle, forward, backward = [float(x) for x in data.decode().split(',')]
                if backward not in [-1, 255]:
                    meas2.append(backward)
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))
        mean_final_dist = np.min(np.array(meas2))


        
        A = COORDS.copy()
        print(O)
        print(A)
        measured_distance_moved = np.linalg.norm(O-A)
        expected_final_dist = mean_initial_dist - measured_distance_moved
        difference = expected_final_dist - mean_final_dist

        
        print("Distance moved = " + str(measured_distance_moved))
        print("Expected final distance = " + str(expected_final_dist))
        print("Measured final distance = " + str(mean_final_dist))
        print("Difference = " + str(difference))
        print("Measured coords = " + str(A))

        norm=np.linalg.norm(A)
        normalised = A/norm

        difference_norm = normalised * difference

        final_coords = A + difference_norm

        print("Adjusted coords = " + str(final_coords))

        ADJUSTMENT = ADJUSTMENT + difference_norm

        for m in meas2:
            plot_measurement(COORDS[0], COORDS[1], HEADING, backward, angle/2, "point", 3)

        if mean_final_dist < 20:
            SOCK.sendto("D".encode(), (UDP_IP, UDP_PORT))
            time.sleep(0.7)
            #HEADING = HEADING + 90
            SOCK.sendto("Z".encode(), (UDP_IP, UDP_PORT))
            time.sleep(0.5)

        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        while True:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                COORDS[0], COORDS[1], HEADING, angle, forward, backward = [float(x) for x in data.decode().split(',')]
                break
            except BlockingIOError:
                pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    main = Main(app)
    main.start()
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
