import socket
import threading
import copy
import time

# Socket variables
UDP_IP = "192.168.137.234"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))

class Communication(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.open = True
        self.data = []

    def run(self):
        while self.open:
            data, address = SOCK.recvfrom(1024)
            decoded = data.decode().split(',')
            self.data.append(tuple(float(x) for x in decoded))

    def reset(self):
        self.data = []

    def sense(self):
        while len(self.data) == 0:
            time.sleep(.02)
        temp = copy.copy(self.data)
        self.data = []
        return temp

    def move(self, power, rotate):
        SOCK.sendto((str(power)+","+str(rotate)).encode(), (UDP_IP, UDP_PORT))

    def stop(self):
        SOCK.sendto("STOP".encode(), (UDP_IP, UDP_PORT))
        SOCK.close()
        self.open = False

    def pause(self):
        SOCK.sendto("PAUSE".encode(), (UDP_IP, UDP_PORT))

    def resume(self):
        SOCK.sendto("RESUME".encode(), (UDP_IP, UDP_PORT))