import socket
import threading

# Socket variables
UDP_IP = "192.168.137.29"
UDP_PORT = 5005
SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
SOCK.bind(("", UDP_PORT))


class Communication(threading.Thread):
    """Class for communicating with Raspberry Pi.

    Attributes:
        running (bool): True when the socket is open.
        measurements (list): Measurements received since last sensed.
    """
    def __init__(self, robot):
        """Initialise Communication object."""
        threading.Thread.__init__(self)
        self.running = True
        self.measurements = []
        self.robot = robot

    def run(self):
        """Thread loop."""
        while self.running:
            data, address = SOCK.recvfrom(1024)  # Get data from socket.
            decoded = data.decode().split(',')  # Split the data.
            self.measurements.append(tuple(float(x) for x in decoded))  # Append to list
        SOCK.close()

    def sense(self):
        """Get measurements since last time sensing."""
        temp = list(self.measurements)
        self.measurements = []
        return temp

    def move(self, power, rotate):
        """Send movement instructions to robot.
        
        Args:
            power (int): Robot speed in range [-255, 255]
            rotate (bool): Rotating or moving in straight line.
        """
        # Send instructions.
        SOCK.sendto((str(power) + "," + str(rotate)).encode(), (UDP_IP, UDP_PORT))

    def stop(self):
        """Send stop instruction to robot."""
        SOCK.sendto("STOP".encode(), (UDP_IP, UDP_PORT))
        self.running = False

    def pause(self):
        """Send pause robot spinning command."""
        SOCK.sendto("PAUSE".encode(), (UDP_IP, UDP_PORT))

    def resume(self):
        """Send resume robot spinning command."""
        SOCK.sendto("RESUME".encode(), (UDP_IP, UDP_PORT))
