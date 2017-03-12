import threading
import time
import math
import random
import copy

import numpy as np
from scipy import stats

from robot import Measurement
from robot import State
import algebra

from PIL import Image, ImageDraw, ImageQt, ImageChops

LIFE = 40
MAX_TRIALS = 1000
MIN_LINE_POINTS = 30
SAMPLE_SIZE = 10
ANGLE_RANGE = 30
RANSAC_TOLERANCE = 5
RANSAC_CONSENSUS = 20

CONSENSUS_PROPORTION = 0.3
SAMPLES = 100
MAX_UNASSIGNED_PROPORTION = 0.0
LANDMARK_RADIUS = 40

ORIGIN = np.array([0, 0])

SENSOR_MEAN = 100.0148733
SENSOR_STDDEV = 3.685631181

DISTANCE_MEAN = 100.1188119
DISTANCE_STDDEV = 8.803385712

ANGLE_MEAN = 90.01
ANGLE_STDDEV = 4.07797744


class Landmark(object):
    def __init__(self, a, b, start=None, end=None, association=None):
        self.position = algebra.closest_point(ORIGIN, a, b)  # Point on line closest to origin
        self.association = association
        self.a = a
        self.b = b
        self.life = LIFE
        self.observed = 1
        self.start = start
        self.end = end

    def distance_to_point(self, point):
        return algebra.line_distance(point, self.a, self.b)

    def distance_to_landmark(self, landmark):
        #linear_distance = algebra.point_distance(self.position, landmark.position)
        angular_distance = abs(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))
        #distance = linear_distance * (1 - angular_distance/2)
        #return distance
        linear_distance = algebra.closestDistanceBetweenLines([self.start, self.end], [landmark.start, landmark.end])
        #angular_distance = abs(math.degrees(math.asin(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))))
        return linear_distance[2] * (1 + angular_distance/2)


def associate_landmarks(new_landmarks, landmarks):
    """Given a set of new landmarks and already known landmarks, match new ones to old ones.

    Args:
        new_landmarks (list): Newly discovered landmarks
        landmarks (list): Known landmarks
    """
    for l in new_landmarks:
        min_distance = 99999
        closest_landmark = None
        # Find the closest known landmark.
        for j in landmarks:
            distance = l.distance_to_landmark(j)
            if distance < min_distance:
                min_distance = distance
                closest_landmark = j
        # If the closest known landmark is within a certain radius, they are the same.
        if min_distance < LANDMARK_RADIUS:
            l.association = closest_landmark
            closest_landmark.observed += 1
            closest_landmark.life = LIFE


def extract_landmarks(points):
    """Function which extracts landmarks from a set of points.

    Args:
        points (list): A set of Measurement objects that detail the robots observations.

    Returns:
        list: A list of discovered landmarks.
    """
    landmarks = []  # Array to store detected landmarks.
    unassigned = list(points)  # Unassigned points.
    no_trials = 0  # Number of trials attempted.
    max_unassigned_points = int(MAX_UNASSIGNED_PROPORTION * len(points))  # Max unassigned points.

    # Look for landmarks while haven't reached max trials, and still enough unassigned points left.
    while no_trials < MAX_TRIALS and len(unassigned) > max_unassigned_points:
        radius = []
        # Randomly select angle, and find points that are close to this angle.
        angle = random.sample(unassigned, 1)[0].angle
        radius = [p for p in unassigned if algebra.angle_distance(p.angle, angle) < ANGLE_RANGE]
        
        if len(radius) >= SAMPLE_SIZE:
            sample = np.array([p.location for p in random.sample(radius, SAMPLE_SIZE)])  # Take a random sample.


            stDevX = np.std(sample[:,0]) 
            stDevY = np.std(sample[:,1]) 

            verticality = stDevY/stDevX #Handle div/0 cases

            isVertical = verticality > 1.5 #adjust based on situation (could be 2.0 or more)

            if isVertical:
                sample = np.fliplr(sample)

            a, b, r_value, p_value, std_err = stats.linregress(sample[:,0], sample[:,1])  # Calculate regression line.

            if isVertical:
                b = -b/a
                a = 1/a

            angle_range = 0
            start = None
            end = None

            # Find the unassigned points that match to this line.
            consensus = []  # Array to store consensus points.
            for point in unassigned:
                if not start:
                    start = point
                    end = point
                # If the point lies close enough to the line.
                if algebra.line_distance(point.location, a, b) < RANSAC_TOLERANCE and algebra.angle_distance(point.angle, angle) < 22.5:
                    consensus.append(point)  # Add it to the consensus points.

                    if algebra.angle_distance(start.angle, point.angle) > angle_range:
                        angle_range = algebra.angle_distance(start.angle, point.angle)
                        end = point
                    elif algebra.angle_distance(point.angle, end.angle) > angle_range:
                        angle_range = algebra.angle_distance(point.angle, end.angle)
                        start = point

            # If there are enough matching points, a landmark has been found.
            if len(consensus) > CONSENSUS_PROPORTION * len(points) and angle_range > ANGLE_RANGE/2:
                # Find new line of regression.
                cartesian = np.array([p.location for p in consensus])  # Convert to array of cartesian points.
                if isVertical:
                    cartesian = np.fliplr(cartesian)
                new_a, new_b, new_r_value, new_p_value, new_std_err = stats.linregress(cartesian[:,0], cartesian[:,1])
                if isVertical:
                    new_b = -b/a
                    new_a = 1/a
                unassigned = [point for point in unassigned if point not in consensus]  # Consensus points are now assigned.
                landmarks.append(Landmark(new_a, new_b, algebra.closest_point(start.location, a, b), algebra.closest_point(end.location, a, b)))  # Add the landmark.
                no_trials = 0  # Found a landmark, so reset trials.
            else:
                no_trials += 1
        else:
            no_trials += 1

    return landmarks


def update_robot(prev, current, landmarks, map):
    """

    """
    # Calculate change in angle and distance moved.
    turned = current.heading - prev.heading
    distance = algebra.point_distance(current.location, prev.location)

    # Calculate probability distribution for angle turned.
    angle_keys = [i for i in range(int(turned-2*ANGLE_STDDEV), int(turned+2*ANGLE_STDDEV))]
    angle_values = stats.norm.pdf(angle_keys, turned, ANGLE_STDDEV)
    angle_probs = {angle_keys[i]: angle_values[i] for i in range(len(angle_keys))}

    # Calculate probability distribution for distance moved
    dstd = DISTANCE_STDDEV * (distance/DISTANCE_MEAN)
    distance_keys = [i for i in range(int(distance-2*dstd), int(distance+2*dstd))]
    distance_values = stats.norm.pdf(distance_keys, distance, dstd)
    distance_probs = {distance_keys[i]: distance_values[i] for i in range(len(distance_keys))}

    max_prob = 0
    delta = (np.zeros(2), 0)

    p_dist = {}

    for dp in distance_probs:  # For each alternative distance.
        for ap in angle_probs:  # For each alternative angle turned.

            # Calculate the position if this angle and distance had been moved.
            alternative_angle = prev.heading+ap
            alternative_location = prev.location + np.array([dp*math.cos(math.radians(alternative_angle)), 
                                                             dp*math.sin(math.radians(alternative_angle))])

            # Difference between expected location and updated location
            delta_angle = alternative_angle - current.heading
            delta_location = alternative_location - current.location

            prob = 0
            for landmark in landmarks:
                # Find where this landmark would be if the robot had moved to this alternate location.
                alternative_landmark = Landmark(*algebra.rotate_line(alternative_location, delta_angle, landmark.a, landmark.b), 
                    algebra.rotate_point(alternative_location, landmark.start, delta_angle),
                    algebra.rotate_point(alternative_location, landmark.end, delta_angle))

                # Find probability of this matching the associated landmark, based on inverse distance.
                distance = alternative_landmark.distance_to_landmark(landmark.association)
                landmark_prob = 1 / ((distance + 1)**2)

                # Multiply these probabilities, and divide by total number of associated landmarks.
                prob += (distance_probs[dp] * angle_probs[ap] * landmark_prob * (1/(1 + math.sin(math.radians(ap)))**2)) / len(landmarks)

            p_dist[(int(alternative_location[0]), int(alternative_location[1]))] = p_dist.get((int(alternative_location[0]), int(alternative_location[1])), 0) + prob

            if prob > max_prob:
                max_prob = prob
                delta = (delta_location, delta_angle)

    probability_sum = sum(p_dist.values())
    if probability_sum == 0:
        probability_sum = 1
    normalised = {key: p_dist[key]/probability_sum for key in p_dist}
    map.plot_prob_dist(normalised)

    return delta

class Slam(threading.Thread):
    def __init__(self, robot, robot_map):
        threading.Thread.__init__(self)
        self.robot = robot
        self.map = robot_map
        self.running = True
        self.landmarks = []
        self.prev = copy.deepcopy(self.robot.adjusted)

    def run(self):
        """SLAM Loop"""
        while self.running:
            # Collect and plot measurements.
            measurements = []
            while len(measurements) < 1000 and self.running:
                new = self.robot.sense()
                for meas in new:
                    if algebra.angle_distance(meas.angle, 0) <= 80:
                        print(str(meas.angle) + " " + str(meas.distance))
                        measurements.append(meas)
                        self.map.plot_measurement(meas)

        while self.running:
            # Start rotating sensor.
            self.robot.resume()
            self.robot.sense()

            # Collect and plot measurements.
            measurements = []
            while len(measurements) < SAMPLES and self.running:
                new = self.robot.sense()
                for meas in new:
                    if meas.distance not in [-1, 255]:# and (len(measurements) == 0 or meas.distance < 1.8*measurements[-1].distance):
                        measurements.append(meas)
                        self.map.plot_measurement(meas)

            # Stop rotating sensor.
            self.robot.pause()

            # Extract and associate landmarks.
            landmarks = extract_landmarks(measurements)
            associate_landmarks(landmarks, self.landmarks)

            """for l in landmarks:
                if l.association:
                    A = None
                    B = None
                    d = 0

                    if algebra.point_distance(l.start, l.end) > d:
                        A = l.start
                        B = l.end
                        d = algebra.point_distance(l.start, l.end)
                    if algebra.point_distance(l.start, l.association.end) > d:
                        A = l.start
                        B = l.association.end
                        d = algebra.point_distance(l.start, l.association.end)
                    if algebra.point_distance(l.association.start, l.end) > d:
                        A = l.association.start
                        B = l.end
                        d = algebra.point_distance(l.association.start, l.end)
                    if algebra.point_distance(l.association.start, l.association.end) > d:
                        A = l.association.start
                        B = l.association.end
                        d = algebra.point_distance(l.association.start, l.association.end)
                    l.association.a = algebra.line_equation(A, B)[0]
                    l.association.b = algebra.line_equation(A, B)[1]
                    l.association.start = A
                    l.association.end = B
                    print(l.association)"""

            # Find most probable position based on matched landmarks, and update robot.
            delta = update_robot(self.prev, self.robot.adjusted, [l for l in landmarks if l.association], self.map)
            self.robot.adjustment.delta(delta[0], delta[1])

            adjusted_landmarks = [Landmark(*algebra.rotate_line(self.robot.adjusted.location, delta[1], landmark.a, landmark.b), 
                    algebra.rotate_point(self.robot.adjusted.location, landmark.start, delta[1]),
                    algebra.rotate_point(self.robot.adjusted.location, landmark.end, delta[1]), 
                    landmark.association) for landmark in landmarks]

            # Add/remove landmarks to array.
            remove = []
            for landmark in adjusted_landmarks:
                # Add new landmarks
                if not landmark.association:
                    self.landmarks.append(landmark)
            
            for landmark in landmarks:
                self.map.plot_landmark(landmark, self.robot.adjusted, (0,0,0,255))

            for landmark in remove:
                self.landmarks.remove(remove)
            
            # Set current state to previous state for next round.
            self.prev = copy.deepcopy(self.robot.adjusted)

            # If close enough to landmark, rotate.
            rotate = False
            for measurement in measurements:
                if algebra.angle_distance(0, measurement.angle) < 10 and measurement.distance < 30:
                    rotate = True

            # Move the robot.
            t = time.time()
            self.robot.move(120, rotate)
            while time.time() - t < 0.8:
                self.robot.sense()
                time.sleep(0.01)
            self.robot.move(0, False)

    def stop(self):
        self.running = False
