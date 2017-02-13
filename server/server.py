import socket
import sys
import math
import threading
import time
import random
import operator
from functools import reduce

import numpy as np
from scipy import stats
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt, ImageChops

# Socket variables
UDP_IP = "10.246.40.233"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

# Robot variables
RUNNING = True  # Robot is running
HEADING = 0  # Robot starts facing right
WIDTH = 1920
HEIGHT = 1080

ADJUSTMENT = np.array([0.0,0.0])


class CameraViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraViewer, self).__init__()

        self.map = Map(WIDTH, HEIGHT)
        self.robot = Robot(WIDTH, HEIGHT)
        self.main = Main(self.map, self.robot)
        self.main.start()

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
        # Convert to greyscale and display
        image = self.map.display()
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
            self.map.clear()

        elif key < 128:
            # Send ASCII key to robot.
            SOCK.sendto(chr(key).encode(), (UDP_IP, UDP_PORT))


class Measurement(object):
    def __init__(self, distance, angle, coords, heading):
        self.distance = distance
        self.angle = (angle + heading) % 360
        self.coords = coords + np.array([WIDTH/2, HEIGHT/2])
        self.end_coords = np.array([self.coords[0]+distance*math.cos(math.radians(self.angle)), 
                                    self.coords[1]+distance*math.sin(math.radians(self.angle))])


class Robot(object):
    def __init__(self, width, height):
        self.dimension = np.array([width, height])
        self.coords = np.array([0.0, 0.0])
        self.coords_adjustment = np.array([0.0, 0.0])
        self.heading = 0.0
        self.heading_adjustment = 0.0

    def get_coords(self):
        return self.coords + self.dimension/2

    def get_adjusted_coords(self):
        return self.coords + self.coords_adjustment + self.dimension/2

    def get_heading(self):
        return self.heading

    def get_adjusted_heading(self):
        return self.heading + self.heading_adjustment

    def set_coords(self, x, y):
        self.coords[0] = x
        self.coords[1] = y

    def set_heading(self, h):
        self.heading = h

    def update_adjustment(self, v):
        self.coords_adjustment += v

    def update_heading_adjustment(self, a):
        self.heading += a


class Map(object):
    def __init__(self, width, height, points=False):
        # Map variables
        self.width = width  # Image width
        self.height = height  # Image height
        self.image = Image.new("RGBA", (WIDTH, HEIGHT), (128,128,128,255))  # The grey background
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space
        self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

        self.points = points
        self.point_radius = 3
        self.robot_radius = 3
        self.arc_angle = 12

    def plot(self, meas):
        objects_draw = ImageDraw.Draw(self.objects)
        space_draw = ImageDraw.Draw(self.space)

        if self.points:
            self.arc_angle = math.degrees(math.asin(self.point_radius/meas.distance))
            objects_draw.ellipse([meas.end_coords[0]-self.point_radius, meas.end_coords[1]-self.point_radius, meas.end_coords[0]+self.point_radius, meas.end_coords[1]+self.point_radius], fill=(0,0,0,255))
        else:
            objects_draw.arc([meas.coords[0]-meas.distance-1, meas.coords[1]-meas.distance-1, meas.coords[0]+meas.distance+1, meas.coords[1]+meas.distance+1], meas.angle-self.arc_angle, meas.angle+self.arc_angle, fill=(0,0,0,255))
        space_draw.pieslice([meas.coords[0]-meas.distance, meas.coords[1]-meas.distance, meas.coords[0]+meas.distance, meas.coords[1]+meas.distance], meas.angle-self.arc_angle, meas.angle+self.arc_angle, fill=(255,255,255,255))

    def plot_state(self, robot):
        self.state = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))
        draw = ImageDraw.Draw(self.state)
        
        # Calculate the x and y lengths for the robot direction
        delta_x = 50 * math.cos(math.radians(robot.get_heading()))
        delta_y = 50 * math.sin(math.radians(robot.get_heading()))
        adjusted_delta_x = 50 * math.cos(math.radians(robot.get_adjusted_heading()))
        adjusted_delta_y = 50 * math.sin(math.radians(robot.get_adjusted_heading()))

        # Plot non-adjusted state
        draw.ellipse((robot.get_coords()[0]-self.robot_radius, robot.get_coords()[1]-self.robot_radius, robot.get_coords()[0]+self.robot_radius, robot.get_coords()[1]+self.robot_radius), fill=(100,0,0,255))
        draw.line((robot.get_coords()[0], robot.get_coords()[1], robot.get_coords()[0]+delta_x, robot.get_coords()[1]+delta_y), fill=(255,0,0,255))

        # Plot adjusted state
        draw.ellipse((robot.get_adjusted_coords()[0]-self.robot_radius, robot.get_adjusted_coords()[1]-self.robot_radius, robot.get_adjusted_coords()[0]+self.robot_radius, robot.get_adjusted_coords()[1]+self.robot_radius), fill=(255,0,0,255))
        draw.line((robot.get_adjusted_coords()[0], robot.get_adjusted_coords()[1], robot.get_adjusted_coords()[0]+adjusted_delta_x, robot.get_adjusted_coords()[1]+adjusted_delta_y), fill=(255,0,0,255))

    def clear(self):
        self.objects = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for obstacle detection
        self.space = Image.new("RGBA", (WIDTH, HEIGHT), (0,0,0,0))  # Image for free space

    def display(self):
        im = Image.alpha_composite(self.image, self.space)
        im = Image.alpha_composite(im, self.objects)
        im = Image.alpha_composite(im, self.state)
        return im

    def display_without_robot(self):
        im = Image.alpha_composite(self.image, self.space)
        im = Image.alpha_composite(im, self.objects)
        return im

def rmsdiff(im1, im2):
    "Calculate the root-mean-square difference between two images"

    h = ImageChops.difference(im1, im2).histogram()

    # calculate rms
    return math.sqrt(reduce(operator.add, map(lambda h, i: h*(i**2), h, range(256))) / (float(im1.size[0]) * im1.size[1]))


class Main(threading.Thread):
    def __init__(self, m, r):
        threading.Thread.__init__(self)

        self.map = m
        self.robot = r

    def run(self):
        #self.ransac_loop()
        while RUNNING:
            self.continuous_measurement()
        SOCK.close()
        return 0

    def incremental_measurement(self):
        meas = []
        while True:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    break
                x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                coords = np.array([x, y])

                self.robot.set_coords(x, y)
                self.robot.set_heading(heading)

                front_measurement = Measurement(forward, angle/2 + 180, coords, heading)
                rear_measurement = Measurement(backward, angle/2, coords, heading)

                if front_measurement.distance not in [-1, 255]:
                    meas.append(front_measurement)
                if rear_measurement.distance not in [-1, 255]:
                    meas.append(rear_measurement)
                self.map.plot_state(self.robot)
            except BlockingIOError:
                pass

        deg = 12
        while deg <= 348:
            s = [m for m in meas if deg-24 <= m.angle < deg]
            if len(s) > 3:
                min_element = s[0]
                for el in s:
                    if el.distance < min_element.distance:
                        min_element = el
                self.map.plot(min_element)
            deg += 24
        meas = []

    def continuous_measurement(self):
        meas = []
        try:
            data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
            decoded = data.decode().split(',')
            if len(decoded) == 1:
                pass
            else:
                x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                coords = np.array([x, y])

                self.robot.set_coords(x, y)
                self.robot.set_heading(heading)

                front_measurement = Measurement(forward, angle/2 + 180, coords, heading)
                rear_measurement = Measurement(backward, angle/2, coords, heading)

                if front_measurement.distance  not in [-1, 255]:
                    self.map.plot(front_measurement)
                if rear_measurement.distance  not in [-1, 255]:
                    self.map.plot(rear_measurement)
                self.map.plot_state(self.robot)
        except BlockingIOError:
            pass

    def rotational_slam(self):
        global RUNNING
        initial_angle = np.copy(self.robot.get_heading())

        count = 0
        meas = []
        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        SOCK.sendto("C".encode(), (UDP_IP, UDP_PORT))
        while count < 1:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    count += 1
                else:
                    x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                    coords = np.array([x, y])

                    self.robot.set_coords(x, y)
                    self.robot.set_heading(heading)

                    front_measurement = Measurement(forward, angle/2 + 180, coords, heading)
                    rear_measurement = Measurement(backward, angle/2, coords, heading)

                    if front_measurement.distance not in [-1, 255]:
                        meas.append(front_measurement)
                    if rear_measurement.distance not in [-1, 255]:
                        meas.append(rear_measurement)
                    self.map.plot_state(self.robot)
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))
        SOCK.sendto("X".encode(), (UDP_IP, UDP_PORT))

        deg = 12
        while deg <= 348:
            s = [m for m in meas if deg-24 <= m.angle < deg]
            if len(s) > 3:
                min_element = s[0]
                for el in s:
                    if el.distance < min_element.distance:
                        min_element = el
                self.map.plot(min_element)
            deg += 24
        meas = []

        time.sleep(0.5)
        SOCK.sendto("D".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.7)
        #HEADING = HEADING + 90
        SOCK.sendto("Z".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.5)
        final_angle = np.copy(self.robot.get_heading())

        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        while True:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    pass
                else:
                    x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                    coords = np.array([x, y])

                    self.robot.set_coords(x, y)
                    self.robot.set_heading(heading)
                    break
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))

        final_angle = np.copy(self.robot.get_heading())
        rotated = (final_angle - initial_angle) % 360
        distribution = {i: 20-abs(i) for i in range(-19, 20)}
        s = sum(distribution.values())
        prob_dist = {k: v/s for k, v in distribution.items()}

        SOCK.sendto("C".encode(), (UDP_IP, UDP_PORT))
        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        count = 0
        meas = []
        while count < 1:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    count += 1
                else:
                    x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                    coords = np.array([x, y])

                    front_measurement = Measurement(forward, angle/2 + 180, coords, heading)
                    rear_measurement = Measurement(backward, angle/2, coords, heading)

                    if front_measurement.distance not in [-1, 255]:
                        meas.append(front_measurement)
                    if rear_measurement.distance not in [-1, 255]:
                        meas.append(rear_measurement)
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))
        SOCK.sendto("X".encode(), (UDP_IP, UDP_PORT))

        distances = {i: 0 for i in range(-19, 20)}
        max_dist = 0
        for key in distances:
            m2 = [Measurement(x.distance, key, x.coords, x.angle) for x in meas]
            temp_map = Map(WIDTH, HEIGHT)
            deg = 12
            while deg <= 348:
                s = [m for m in m2 if deg-24 <= m.angle < deg]
                if len(s) > 3:
                    min_element = s[0]
                    for el in s:
                        if el.distance < min_element.distance:
                            min_element = el
                    temp_map.plot(min_element)
                deg += 24

            dist = rmsdiff(temp_map.display_without_robot(), self.map.display_without_robot())
            print(dist)
            if dist > max_dist:
                max_dist = dist
                distances[key] = dist

        dist_adjusted = {k: max_dist - distances[k] for k in distances}
        s = sum(dist_adjusted.values())
        dist_prob_dist = {k: v/s for k,v in dist_adjusted.items()}

        multiplied = {i: dist_prob_dist[i]*prob_dist[i] for i in range(-19,20)}
        max_prob = 0
        adjustment = 0
        for key, value in multiplied.items():
            if value > max_prob:
                max_prob = value
                adjustment = key

        print(adjustment)
        self.robot.update_adjustment(adjustment)


    def slam(self):
        global RUNNING
        measurements = []
        O = np.copy(self.robot.get_coords())
        while len(measurements) < 100 and RUNNING:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    pass
                else:
                    x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                    coords = np.array([x, y])

                    self.robot.set_coords(x, y)
                    self.robot.set_heading(heading)

                    meas = Measurement(backward, angle/2, coords, heading)

                    if meas.distance not in [-1, 255]:
                        measurements.append(meas)
                        self.map.plot(meas)
                        self.map.plot_state(self.robot)
            except BlockingIOError:
                pass

        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))  # Stop the robot
        mean_initial_dist = np.min(np.array([m.distance for m in measurements]))

        print("Initial distance = " + str(mean_initial_dist))

        SOCK.sendto("W".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.5)
        SOCK.sendto("Z".encode(), (UDP_IP, UDP_PORT))
        time.sleep(0.5)

        measurements_after=[]
        SOCK.sendto("R".encode(), (UDP_IP, UDP_PORT))
        while len(measurements_after) < 100 and RUNNING:
            try:
                data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
                decoded = data.decode().split(',')
                if len(decoded) == 1:
                    pass
                else:
                    x, y, heading, angle, forward, backward = [float(x) for x in decoded]
                    coords = np.array([x, y])

                    self.robot.set_coords(x, y)
                    self.robot.set_heading(heading)

                    meas = Measurement(backward, angle/2, coords, heading)

                    if meas.distance not in [-1, 255]:
                        measurements_after.append(meas)
                        self.map.plot_state(self.robot)
            except BlockingIOError:
                pass
        SOCK.sendto("P".encode(), (UDP_IP, UDP_PORT))
        mean_final_dist = np.min(np.array([m.distance for m in measurements_after]))


        
        A = np.copy(self.robot.get_coords())
        direction = O-A
        measured_distance_moved = np.linalg.norm(direction)
        expected_final_dist = mean_initial_dist - measured_distance_moved
        difference = expected_final_dist - mean_final_dist

        
        print("Distance moved = " + str(measured_distance_moved))
        print("Expected final distance = " + str(expected_final_dist))
        print("Measured final distance = " + str(mean_final_dist))
        print("Difference = " + str(difference))
        print("Measured coords = " + str(A))

        normalised = direction/measured_distance_moved

        difference_norm = normalised * difference

        final_coords = A + difference_norm

        print("Adjusted coords = " + str(final_coords))

        self.robot.update_adjustment(difference_norm)

        for m in measurements_after:
            m.coords = self.robot.get_adjusted_coords()
            self.map.plot(m)

        if mean_final_dist < 30:
            self.rotational_slam()


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())
