import threading
import time
import math
import copy
import random

import numpy as np
from scipy import stats
from numpy.linalg import norm

from robot import Measurement
from robot import State

from PIL import Image, ImageDraw, ImageQt, ImageChops

LIFE = 40
MAX_TRIALS = 1000
MIN_LINE_POINTS = 30
MAX_SAMPLE = 10
ANGLE_RANGE = 30
RANSAC_TOLERANCE = 5
RANSAC_CONSENSUS = 30
ORIGIN = np.array([1920/2, 1080/2])


class Landmark(object):
    def __init__(self, a, b):
        self.position = closest_point(ORIGIN, a, b)  # Point on line closest to origin
        self.a = a
        self.b = b
        self.life = LIFE

    def distance(self, landmark):
        return 1/((norm(self.position - landmark.position) + 1)**2)


def line_intersection(state, a1, b1):
    a2 = math.sin(math.radians(state.heading))/math.cos(math.radians(state.heading))
    b2 = state.location[1] - a2*state.location[0]

    x = (b2-b1)/(a1-a2)
    y = a2*x + b2

    return np.array([x, y])


def closest_point(point, a, b):
    c = b
    new_b = -1

    x = (new_b*(new_b*point[0] - a*point[1]) - a*c)/(a**2 + new_b**2)
    y = (a*(-new_b*point[0] + a*point[1]) - new_b*c)/(a**2 + new_b**2)

    return np.array([x, y])


class Landmarks(object):
    def __init__(self):
        pass

    def associate_landmarks(self, new_landmarks, old_landmarks, robot):
        result = []
        combine = new_landmarks
        for l in new_landmarks:
            least_distance = 99999
            assoc = None
            s = robot.adjusted()
            for j in old_landmarks:
                p1 = line_intersection(s, l.a, l.b)
                p2 = line_intersection(s, j.a, j.b)
                distance = norm(p1-p2)
                if distance < least_distance:
                    least_distance = distance
            if least_distance < 30:
                j.total_times_observed += 1
                j.error = least_distance

                p1 = line_intersection(s, l.a, l.b)
                p2 = line_intersection(s, j.a, j.b)

                vector = p1 - p2
                print(vector)
                robot.adjustment.delta(delta_location=vector)

                combine.remove(l)
        return old_landmarks + combine


    def extract_landmarks(self, points, position):
        # Arrays to store a and b values of seen lines equations
        equations = []
        corresponding_points = []

        no_trials = 0
        while no_trials < MAX_TRIALS and len(points)-len(corresponding_points) > MIN_LINE_POINTS:
            free_points = [p for p in points if p not in corresponding_points]
            temp = 0
            angle = random.sample(free_points, 1)[0].angle
            possible_points = [p for p in free_points if (p.angle-angle)%360 < ANGLE_RANGE]
            if len(possible_points) <= 5:
                continue
            selected_points = random.sample(possible_points, min(len(possible_points), MAX_SAMPLE))
            cartesian_points = np.array([p.cartesian() for p in selected_points])

            slope, intercept, r_value, p_value, std_err = stats.linregress(cartesian_points[:,0], cartesian_points[:,1])
            consensus_points = []
            p1 = np.array([0, intercept])
            p2 = np.array([1, slope+intercept])
            for point in free_points:
                p3 = point.cartesian()
                if norm(np.cross(p2-p1, p1-p3))/norm(p2-p1) < RANSAC_TOLERANCE:
                    consensus_points.append(point)
            if len(consensus_points) > RANSAC_CONSENSUS and r_value > 0.75:
                print(r_value)
                consensus_cartesian_points = np.array([p.cartesian() for p in consensus_points])
                slope, intercept, r_value, p_value, std_err = stats.linregress(consensus_cartesian_points[:,0], consensus_cartesian_points[:,1])
                corresponding_points.extend(consensus_points)
                equations.append((slope,intercept))
                no_trials = 0
            else:
                no_trials += 1

        landmarks = []
        for a, b in equations:
            landmarks.append(Landmark(a, b, position))

        return landmarks


class Slam(threading.Thread):
    def __init__(self, robot, robot_map):
        threading.Thread.__init__(self)
        self.robot = robot
        self.map = robot_map
        self.running = True
        self.landmarks = []

    def run(self):
        l = Landmarks()
        while self.running:
            self.robot.resume()
            m = []
            while len(m) < 100 and self.running:
                meas = self.robot.sense()
                m.extend(meas)
                self.map.plot_measurement(meas[0])
                self.map.plot_measurement(meas[1])
                m = [x for x in m if x.distance not in [-1, 255]]
            self.robot.pause()
            new_landmarks = l.extract_landmarks(m, self.robot.state.location)
            print(len(new_landmarks))
            self.landmarks = l.associate_landmarks(new_landmarks, self.landmarks, self.robot)

            draw = ImageDraw.Draw(self.map.objects)
            for landmark in self.landmarks:
                draw.line(((0, landmark.b), (1920, 1920*landmark.a+landmark.b)), fill=(0,0,0,255))
            
            t = time.time()
            self.robot.move(255, False)
            while time.time() - t < 1:
                self.robot.update()
            self.robot.move(0, False)




    def stop(self):
        self.running = False