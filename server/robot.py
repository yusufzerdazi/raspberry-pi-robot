import communication
import copy
import math
import numpy as np

class State(object):
    def __init__(self, origin=np.zeros(2), location=np.zeros(2), heading=0.0):
        self.origin = origin
        self.location = location
        self.heading = heading

    def update(self, location, heading):
        self.location = location + self.origin
        self.heading = heading

    def delta(self, delta_location=np.zeros(2), delta_heading=0.0):
        self.location += delta_location
        self.heading += delta_heading


class Measurement(object):
    def __init__(self, state, angle, distance):
        self.state = state
        self.angle = angle
        self.distance = distance

    def cartesian(self):
        theta = math.radians(self.angle + self.state.heading)
        dx = self.distance * math.cos(theta)
        dy = self.distance * math.sin(theta)
        return self.state.location + np.array([dx, dy])


class Robot(object):
    def __init__(self, width, height):
        self.origin=np.array([width/2, height/2])
        self.state = State(origin=self.origin)
        self.adjustment = State()
        self.communication = communication.Communication()

    def adjusted(self):
        return State(origin=self.origin, location=self.state.location+self.adjustment.location, heading=self.state.heading+self.adjustment.heading)

    def update(self):
        x, y, heading, angle, front, rear = self.communication.sense()
        location = np.array([x, y])
        self.state.update(location, heading)
        return (angle, front, rear)

    def sense(self):
        result = []
        angle, front, rear = self.update()
        result.append(Measurement(self.state, angle/2, front))
        result.append(Measurement(self.state, angle/2 + 180, rear))
        return result

    def move(self, speed, rotate):
        self.communication.move(speed, rotate)

    def stop(self):
        self.communication.stop()

    def pause(self):
        self.communication.pause()

    def resume(self):
        self.communication.resume()