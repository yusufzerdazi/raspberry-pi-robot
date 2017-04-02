import threading
import time
import math
import util
import comm
import numpy as np

ANGLE_PER_ENCODER = 0.109236273
CM_PER_ENCODER = 0.022114184


class Encoders(object):
    def __init__(self):
        self.tol = 20
        self.encoders = np.array([0, 0, 0])
        self.actual = np.array([0, 0, 0])

    def update(self):
        current = np.array(util.motors())  # Get encoder values.
        sign = np.sign(current - self.encoders)  # Find direction of rotation.
        tolerance = np.invert(np.isclose(current, self.actual, rtol=0, atol=self.tol))  # Determine which are outside the range.
        self.actual[tolerance] = (current - sign * self.tol)[tolerance]  # Update the encoders that are outside the range.
        self.encoders = current  # Update the encoder values.


class Sensor(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.lock = threading.Lock()
        self.speed = 60
        self.paused = True  # Start out paused
        self.running = True  # Robot is sensing
        self.state = threading.Condition()

    def run(self):
        while self.running:
            with self.state:
                if self.paused:
                    self.state.wait()  # Block until notified
            util.rotate(90, self.speed)
            util.rotate(-90, self.speed)
            util.rotate(0, self.speed)

    def resume(self):
        with self.state:
            self.paused = False
            self.state.notify()  # Unblock self if waiting

    def pause(self):
        with self.state:
            self.paused = True  # Make self block and wait

    def stop(self):
        with self.state:
            self.running = False
            self.state.notify()  # Unblock self if waiting


class State(object):
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.left, self.right = 0.0, 0.0
        self.heading = 0.0

    def update(self, encoders):
        # Calculate change in encoder values
        """dleft = left - self.left
        dright = right - self.right
        distance = (dleft + dright) / 2

        if dleft * dright > 0:
            self.x = self.x + distance * math.cos(math.radians(self.heading)) * CM_PER_ENCODER
            self.y = self.y + distance * math.sin(math.radians(self.heading)) * CM_PER_ENCODER
        else:
            theta = ANGLE_PER_ENCODER * (dright - dleft)
            self.heading = self.heading + theta

        self.left, self.right = left, right"""

        sensor, left, right = encoders

        left_delta = (left - self.left)
        right_delta = (right - self.right)

        if left_delta == right_delta:
            self.x = self.x + left_delta * math.cos(math.radians(self.heading)) * CM_PER_ENCODER
            self.y = self.y + right_delta * math.sin(math.radians(self.heading)) * CM_PER_ENCODER
        else:
            wd = (right_delta - left_delta) * ANGLE_PER_ENCODER
            R = ((left_delta + right_delta) * wd) / (CM_PER_ENCODER * (right_delta - left_delta)**2)

            self.x = self.x + R * math.sin(math.radians(wd + self.heading)) - R * math.sin(math.radians(self.heading))
            self.y = self.y - R * math.cos(math.radians(wd + self.heading)) + R * math.cos(math.radians(self.heading))
            self.heading = (self.heading + wd)

        self.left = left
        self.right = right


class Robot(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = True
        self.comm = comm.Communication()
        self.sensor = Sensor()
        self.state = State()
        self.encoders = Encoders()
        self.sensor.start()

    def run(self):
        while self.running:
            # Update sensor values
            util.update()
            self.encoders.update()
            self.state.update(self.encoders.actual)

            # Get recieved commands
            sensing = self.comm.recieve.sensing
            power = self.comm.recieve.power
            rotate = self.comm.recieve.rotate
            self.running = self.comm.recieve.running

            # Update robot based on commands
            if sensing and self.sensor.paused:
                self.sensor.resume()
            elif (not sensing) and (not self.sensor.paused):
                self.sensor.pause()
            util.move(power, rotate)

            # Send data
            sensors = util.sensors()
            self.comm.send(self.state.x, self.state.y, self.state.heading, *sensors)

            # Sleep so sensors have time to update
            time.sleep(.02)

        self.sensor.stop()
        self.comm.stop()

    def stop(self):
        self.running = False


if __name__ == "__main__":
    # Start the robot thread
    robot = Robot()
    robot.start()