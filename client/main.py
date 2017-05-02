import threading
import time
import math
import util
import comm
import numpy as np

ANGLE_PER_ENCODER = 0.0983126457 #0.109236273
CM_PER_ENCODER = 0.02295788632 #0.022114184


class Encoders(object):
    def __init__(self):
        self.tol = 0
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
        self.speed = 50
        self.paused = True  # Start out paused
        self.running = True  # Robot is sensing
        self.state = threading.Condition()
        self.right = True

    def run(self):
        while self.running:
            with self.state:
                if self.paused:
                    self.state.wait()  # Block until notified
            util.rotate(90, self.speed)
            self.right = False
            util.rotate(-90, self.speed)
            self.right = True
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
        sensor, left, right = encoders

        left_delta = (left - self.left)
        right_delta = (right - self.right)

        if np.sign(left_delta) == np.sign(right_delta):#left_delta == right_delta:
            self.x = self.x + left_delta * math.cos(math.radians(self.heading)) * CM_PER_ENCODER
            self.y = self.y + right_delta * math.sin(math.radians(self.heading)) * CM_PER_ENCODER
        else:
            #wd = (right_delta - left_delta) * ANGLE_PER_ENCODER
            #R = ((left_delta + right_delta) * wd) / (CM_PER_ENCODER * (right_delta - left_delta)**2)

            #self.x = self.x + R * math.sin(math.radians(wd + self.heading)) - R * math.sin(math.radians(self.heading))
            #self.y = self.y - R * math.cos(math.radians(wd + self.heading)) + R * math.cos(math.radians(self.heading))
            #self.heading = (self.heading + wd)

            sgn = np.sign(right_delta)
            avg = abs(left_delta) + abs(right_delta)
            self.heading += sgn * avg * ANGLE_PER_ENCODER


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
        count = 0
        while self.running:
            # Update sensor values
            util.update()
            self.encoders.update()
            front, rear = util.sensors()
            r = self.sensor.right
            #if count < 5000:
            #    if front not in [-1, 255]:
            #        print str(count) + "," + str(abs(int(self.encoders.actual[0])/2)) + "," + str(front)
            #        count += 1
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
            self.comm.send(self.state.x, self.state.y, self.state.heading, self.encoders.actual[0], front, rear)

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