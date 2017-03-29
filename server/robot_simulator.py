import time
import math
import numpy as np
from server import algebra
import random
import threading
from server.robot import State


class Communication(threading.Thread):
    def __init__(self, robot, map):
        threading.Thread.__init__(self)

        self.state = State()
        self.adjustment = State()
        self.adjusted = self.state + self.adjustment
        self.current = time.time()

        self.rotating = False
        self.speed = 0
        self.heading = 0.0
        self.angle = 0.0
        self.x = 0.0
        self.y = 0.0

        self.error = [0,0]
        self.measurements = []
        self.running = True

        self.robot = robot
        self.map = map

    def run(self):
        while self.running:
            if time.time() - self.current < 0.02:
                time.sleep(0.02)

            new = time.time()
            delta_time = new - self.current

            if self.rotating:
                self.heading += self.speed * delta_time * 0.4
            else:
                distance = self.speed * delta_time * 0.1
                self.x += distance * math.cos(math.radians(self.heading))
                self.y += distance * math.sin(math.radians(self.heading))

            location = np.array([self.x, self.y])
            moved = location - self.state.location
            delta = algebra.rotate_point(np.zeros(2), moved, self.adjustment.heading) - moved

            # Update the state and adjustment
            self.state.update(location, self.heading)
            self.adjustment.delta(delta)
            self.adjusted = self.state + self.adjustment

            t = self.current
            m = 180
            angle = m - abs(t * 150 % (2 * m) - m) - 90

            RY = 80
            RX = 90

            if 90 < (angle + self.heading) % 360 <= 270:
                FA = abs((RX + self.x) / math.cos(math.radians(angle + self.heading))) + 4 * random.random()
                RA = abs((RX - self.x) / math.cos(math.radians(angle + self.heading))) + 4 * random.random()
            else:
                FA = abs((RX - self.x) / math.cos(math.radians(angle + self.heading))) + 4 * random.random()
                RA = abs((RX + self.x) / math.cos(math.radians(angle + self.heading))) + 4 * random.random()

            if 180 < (angle + self.heading) % 360 <= 360:
                FB = abs((RY + self.y) / math.sin(math.radians(angle + self.heading))) + 4 * random.random()
                RB = abs((RY - self.y) / math.sin(math.radians(angle + self.heading))) + 4 * random.random()
            else:
                FB = abs((RY - self.y) / math.sin(math.radians(angle + self.heading))) + 4 * random.random()
                RB = abs((RY + self.y) / math.sin(math.radians(angle + self.heading))) + 4 * random.random()

            F = min(FA, FB)
            R = min(RA, RB)

            front = F if (F < 100) else 255
            rear = R if (R < 100) else 255

            # Append measurements for front and rear sensors.
            measurements = [m for m in self.robot.update((self.x, self.y, self.heading, (angle * 2), front, rear)) if m.distance < 255]
            self.map.plot_measurements(measurements)

            self.current = new

    def sense(self):
        """Access the measurements recieved since the last sense, update the robot's state, and return
        them as a list of Measurement objects.

        Returns:
            list: List of measurements.
        """
        result = list(self.measurements)
        self.measurements = []
        return result

    def senses(self, n):
        measurements = []
        while len(measurements) < n:
            new = self.sense()
            measurements.extend([m for m in new if m.distance not in [-1, 255]])
        return measurements

    def move(self, speed, rotate):
        """Sends speed and direction instructions to robot"""
        self.current = time.time()
        self.speed = speed
        self.rotating = rotate

    def stop(self):
        """Tells robot to stop"""
        self.running = False

    def pause(self):
        """Tells robot to pause sensing"""
        pass

    def resume(self):
        """Tells robot to resume sensing"""
        pass
