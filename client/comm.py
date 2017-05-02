import socket
import threading


class Recieve(threading.Thread):
    def __init__(self, sock):
        threading.Thread.__init__(self)
        self.sock = sock
        self.running = True
        self.power = 0
        self.rotate = False
        self.sensing = True

    def run(self):
        while self.running:
            command, address = self.sock.recvfrom(1024)
            decoded = command.decode().split(',')
            if len(decoded) > 1:
                self.power = int(decoded[0])
                self.rotate = (decoded[1] == "True")
            elif command == "STOP":
                self.running = False
            elif command == "PAUSE":
                self.sensing = False
            elif command == "RESUME":
                self.sensing = True


class Communication(object):
    def __init__(self):
        # Set up socket
        self.udp_ip = "10.245.131.89"#10.245.131.211"  # UDP IP Address
        self.udp_port = 5005  # UDP Port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.udp_port))

        # For recieving commands
        self.recieve = Recieve(self.sock)
        self.recieve.start()

    def send(self, *args):
        self.sock.sendto((",".join([str(arg) for arg in args])).encode(), (self.udp_ip, self.udp_port))

    def stop(self):
        self.sock.close()
