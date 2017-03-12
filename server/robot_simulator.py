import time
import math
import numpy as np
import algebra
import random
from robot import State
from robot import Measurement

class Robot(object):
    def __init__(self):
        self.state = State()
        self.adjustment = State()
        self.adjusted = self.state + self.adjustment
        self.start = time.time()
        self.current = time.time()

        self.rotating = False
        self.speed = 0
        self.heading = 0.0
        self.angle = 0.0
        self.x = 0.0
        self.y = 0.0

        self.error = [0,0]

    def sense(self):
        """Access the measurements recieved since the last sense, update the robot's state, and return
        them as a list of Measurement objects.

        Returns:
            list: List of measurements.
        """
        if time.time() - self.current < 0.002:
            pass#time.sleep(0.0002)

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

        if self.speed:
            pass
            #self.error[0] = self.error[0] + random.random()-0.5
            #self.error[1] = self.error[1] + random.random()-0.5

        t = self.current
        m = 180
        angle = m - abs(t*1500 % (2*m) - m) - 90
        dxr = 80 - self.x - self.error[0] + 4*random.random()
        dxl = 80 + self.x + self.error[0] + 4*random.random()
        dyb = 80 - self.y - self.error[1] + 4*random.random()
        dyt = 80 + self.y + self.error[1] + 4*random.random()

        f_distance = 0
        r_distance = 0
        if 0 < (angle + self.heading) % 360 < 180:
            f_distance = abs(dyb/math.sin(math.radians(angle+self.heading)))
            r_distance = abs(dyt/math.sin(math.radians(angle+self.heading+180)))
            
        else:
            f_distance = abs(dyt/math.sin(math.radians(angle+self.heading)))
            r_distance = abs(dyb/math.sin(math.radians(angle+self.heading+180)))

        """if 90 < (angle + self.heading) % 360 < 270:
            f_distance = min(f_distance, abs(dxl/math.cos(math.radians(angle+self.heading))))
            r_distance = min(r_distance, abs(dxr/math.cos(math.radians(angle+self.heading+180))))
        else:
            f_distance = min(f_distance, abs(dxr/math.cos(math.radians(angle+self.heading))))
            r_distance = min(r_distance, abs(dxl/math.cos(math.radians(angle+self.heading+180))))"""

        front = f_distance if (f_distance < 100) else 255# and np.sign(math.cos(math.radians(angle + self.heading))) > 0) else 255
        rear = r_distance if (r_distance < 100) else 255# and np.sign(math.cos(math.radians(angle + self.heading))) > 0) else 255

        # Append measurements for front and rear sensors.
        measurements = []
        measurements.append(Measurement(self.adjusted, angle % 360, front))
        measurements.append(Measurement(self.adjusted, (angle + 180) % 360, rear))

        self.current = new

        return measurements

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
        pass

    def pause(self):
        """Tells robot to pause sensing"""
        pass

    def resume(self):
        """Tells robot to resume sensing"""
        pass
