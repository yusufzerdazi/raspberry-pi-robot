import socket
import sys
import math
import threading
import msvcrt
from PyQt5 import QtCore, QtGui, QtWidgets
from PIL import Image, ImageDraw, ImageQt

IMAGE = Image.new("L", (512, 512), "grey")
UDP_IP = "192.168.0.41"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)
LINES = []


class Robot(object):
    def __init__(self, x, y):
        self.initial_x = x
        self.initial_y = y
        self.x = x
        self.y = y

    def update_position(self, x, y):
        self.x = self.initial_x + x
        self.y = self.initial_y + y


class Map(object):
    def __init__(self, width, height):
        self.robot = Robot(width/2, height/2)
        self.image = Image.new("L", (width, height), "grey")

    def get_image(self):
        return self.image

    def update_map(self, distance, angle):
        if distance != 255:
            draw = ImageDraw.Draw(self.image)
            angle_radians = 0.0174533 * (angle + 270)

            x_distance = distance * math.cos(angle_radians)
            y_distance = distance * math.sin(angle_radians)

            LINES.append((self.robot.x, self.robot.y, self.robot.x + int(x_distance), self.robot.y + int(y_distance)))
            draw.line((self.robot.x, self.robot.y, self.robot.x + int(x_distance), self.robot.y + int(y_distance)), fill=255)
            draw.point((self.robot.x + int(x_distance), self.robot.y + int(y_distance)), fill=0)

robot_map = Map(750, 950)


class CameraViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super(CameraViewer, self).__init__()

        self.imageLabel = QtWidgets.QLabel()
        self.imageLabel.setBackgroundRole(QtGui.QPalette.Base)
        self.imageLabel.setScaledContents(True)

        self.scrollArea = QtWidgets.QScrollArea()
        self.scrollArea.setWidget(self.imageLabel)
        self.setCentralWidget(self.scrollArea)

        self.setWindowTitle("Image Viewer")
        self.resize(500, 500)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.open)
        timer.start(33)  # 30 Hz

    def open(self):
        # Get data and display
        image = ImageQt.ImageQt(robot_map.image)
        self.imageLabel.setPixmap(QtGui.QPixmap.fromImage(image))
        self.imageLabel.adjustSize()


def get_robot_data():
    while True:
        draw = ImageDraw.Draw(robot_map.image)
        data, address = SOCK.recvfrom(1024)  # buffer size is 1024 bytes
        distance, angle, x, y = [float(x) for x in data.decode().split(', ')]
        print([distance, angle, x, y])
        robot_map.robot.update_position(x, y)

        #if len(LINES) > 40:
        #    l = LINES.pop(0)
        #    draw.line(l, fill=125)

        #for i in range(len(LINES)):
        #    draw.line(LINES[i], fill=int(125.0+125.0*(float(i)/float(len(LINES)))))
        robot_map.update_map(distance, angle)


def start():
    app = QtWidgets.QApplication(sys.argv)
    cv = CameraViewer()
    cv.show()
    sys.exit(app.exec_())


#if __name__ == "__main__":
#    threading.Thread(target=start).start()
#    threading.Thread(target=get_robot_data).start()


while True:
    try:
        data, address = SOCK.recvfrom(1024)
        print(data)
    except BlockingIOError as ex:
        pass
    key = msvcrt.getch().decode().upper()
    if key:
        print(key + " pressed.")
        SOCK.sendto(key.encode(), (UDP_IP, UDP_PORT))
        if key == "X":
            SOCK.close()
            break
