import socket

# Socket variables
UDP_IP = "192.168.137.3"  # UDP IP Address
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))
SOCK.setblocking(0)

class Communication(object):
    def __init__(self):
        self.open = True

    def sense(self):
        decoded = []
        while len(decoded) == 0:
            try:
                data, address = SOCK.recvfrom(1024)
                decoded = data.decode().split(',')
            except socket.error:
                pass
        return (float(x) for x in decoded)

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