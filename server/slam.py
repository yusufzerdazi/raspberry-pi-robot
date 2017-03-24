import threading
import time
import math
import random
import copy

import numpy as np
from scipy import stats

from robot import Measurement
from robot import State
from robot import RobotState
import algebra

from PIL import Image, ImageDraw, ImageQt, ImageChops

LIFE = 40
MAX_TRIALS = 1000
MIN_LINE_POINTS = 30
SAMPLE_SIZE = 10
ANGLE_RANGE = 30
RANSAC_TOLERANCE = 10
RANSAC_CONSENSUS = 20

CONSENSUS_PROPORTION = 0.2
SAMPLES = 150
MAX_UNASSIGNED_PROPORTION = 0.0
LANDMARK_RADIUS = 10

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
        #angular_distance = abs(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))
        #distance = linear_distance * (1 - angular_distance/2)
        #return distance
        angle = min(abs(math.asin(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))), abs(math.asin(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))))
        p = angle/(math.pi/2)
        linear_distance = algebra.closestDistanceBetweenLines([self.start, self.end], [landmark.start, landmark.end])
        angular_distance = 50 * math.tan(angle)
        #angular_distance = abs(math.degrees(math.asin(np.cross(algebra.normal(self.a), algebra.normal(landmark.a)))))
        return (1-p) * linear_distance[2] + p * angular_distance#linear_distance[2] * (1 + angular_distance/2)


def collect_measurements_simple(communication, robot, robot_map, samples=SAMPLES, plot=True, spurious=False):
    # Start rotating sensor.
    communication.resume()

    # Reset robot's measurements
    robot.reset()

    # Collect measurements.
    while(len([m for m in robot.measurements if (int(m.distance) != 255 or spurious)])) < samples:
        time.sleep(.02)

    # Capture measurements.
    measurements = [m for m in robot.measurements if (int(m.distance) != 255 or spurious)]

    # Plot measurements.
    if plot:
        robot_map.plot_measurements(measurements)

    # Stop rotating sensor.
    communication.pause()

    # Sort and return measurements
    return sorted(measurements, key=lambda measurement: measurement.angle)


def collect_measurements_metered(communication, robot, robot_map, plot=True, windows=30):
    measurements = collect_measurements_simple(communication, robot, robot_map, plot=False, spurious=True)
    result = []

    width = 360/windows
    radius = width/2
    centre = 0

    while centre % 360 == centre:
        distances = [m.distance for m in measurements if algebra.angle_distance(m.angle, centre) < radius]
        if len(distances) > 0:
            measurement = Measurement(robot.adjusted, centre, min(distances))
            robot_map.plot_measurement(measurement)
            result.append(measurement)
        centre += width

    return result


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


def extract_landmarks_clustered(points, labels):
    landmarks = []
    unassigned = list(points)

    for label in labels:
        sample = np.array([points[i].location for i in range(len(labels)) if labels[i] == label])

        # Calculate the standard deviation of the sample.
        std_x = np.std(sample[:,0]) 
        std_y = np.std(sample[:,1]) 

        # If the line is close to vertical, find the regression line in terms of x.
        isVertical = False
        if(std_x == 0 or std_y/std_x > 1):
            sample = np.fliplr(sample)
            isVertical = True

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
            if algebra.line_distance(point.location, a, b) < RANSAC_TOLERANCE:
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
    return landmarks


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
        
        if len(radius) >= 3:
            sample = np.array([p.location for p in random.sample(radius, 3)])  # Take a random sample.

            # Calculate the standard deviation of the sample.
            std_x = np.std(sample[:,0]) 
            std_y = np.std(sample[:,1]) 

            # If the line is close to vertical, find the regression line in terms of x.
            isVertical = False
            if(std_x == 0 or std_y/std_x > 1):
                sample = np.fliplr(sample)
                isVertical = True

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
                if algebra.line_distance(point.location, a, b) < RANSAC_TOLERANCE:# and algebra.angle_distance(point.angle, angle) < 22.5:
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
    distance_keys = [i for i in range(int(distance-2*DISTANCE_STDDEV), int(distance+2*DISTANCE_STDDEV))]
    distance_values = stats.norm.pdf(distance_keys, distance, DISTANCE_STDDEV)
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
                landmark_prob = 1 / ((distance/100 + 1)**2)

                # Multiply these probabilities, and divide by total number of associated landmarks.
                prob += (distance_probs[dp] * angle_probs[ap] * landmark_prob)# * (1/(1 + math.sin(math.radians(ap)))**2)) / len(landmarks)

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
    """Class which contains the main SLAM loop.
    
    Attributes:
        robot (robot.Robot): Robot object.
        map (map.Map): Occupancy grid.
        running (bool): Thread running.
        landmarks (array): List of unique landmarks.
        prev (robot.State): Copy of previous robot state.
        paused (bool): SLAM paused.
        paused_cond (threading.Condition): Condition for managing paused state.
    """
    def __init__(self, robot, occupancy, communication, controlled):
        """Initialise SLAM object.

        Args:
            robot (robot.Robot): Robot object.
            occupancy (robot.Map): Occupancy grid.
            controlled (bool): Whether to run SLAM automatically or wait for user input.
        """
        threading.Thread.__init__(self)
        self.robot = robot
        self.map = occupancy
        self.communication = communication
        self.running = True
        self.controlled = controlled
        self.landmarks = []
        self.prev = copy.deepcopy(self.robot)

        self.paused = False
        self.pause_cond = threading.Condition(threading.Lock())

    def run(self):
        """SLAM Loop"""
        while self.running:
            # Pause after every SLAM iteration, if not automatic.
            if self.controlled:
                self.pause()

            # Stop thread running while paused.
            with self.pause_cond:
                while self.paused:
                    self.pause_cond.wait()

            # If the run was stopped while paused, need to break the loop.
            if not self.running:
                break

            # Collect measurements.
            measurements = collect_measurements_simple(self.communication, self.robot, self.map)

            # Extract and associate landmarks.
            landmarks = extract_landmarks(measurements)
            associate_landmarks(landmarks, self.landmarks)

            """for l in landmarks:
                if l.association:
                    self.map.plot_landmark(l, self.robot.adjusted, (255,0,0,255))
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
                    l.association.end = B#"""

            # Find most probable position based on matched landmarks, and update robot.
            delta = update_robot(self.prev.adjusted, self.robot.adjusted, [l for l in landmarks if l.association], self.map)
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
            
            #self.map.landmarks = Image.new("RGBA", (1920, 1080), (0,0,0,0))  # Image for landmarks
            for landmark in self.landmarks:
                self.map.plot_landmark(landmark, self.robot.adjusted, (0,0,0,255))

            for landmark in landmarks:
                if landmark.association:
                    if landmark.distance_to_landmark(landmark.association) > 5:
                        self.map.plot_landmark(landmark, self.robot.adjusted, (255,0,0,255))

            for landmark in remove:
                self.landmarks.remove(remove)

            # Set current state to previous state for next round.
            self.prev = copy.deepcopy(self.robot)

            # Move robot, if running automatically.
            if not self.controlled:
                self.move(measurements)
                
    def move(self, measurements):
        """Move the robot, depending on whether there is an obstacle in front of it.
        
        Args:
            measurements (array): List of measurements.
        """
        # If close enough to object, rotate, otherwise move straight.
        rotate = (len([measurement for measurement in measurements if algebra.angle_distance(0, measurement.angle) < 10 and measurement.distance < 30]) > 0)
        t = time.time()
        self.communication.move(150, rotate)
        while time.time() - t < 0.8:
            self.communication.sense()
            time.sleep(0.01)
        self.communication.move(0, False)

    def pause(self):
        if not self.paused:
            self.paused = True
            self.pause_cond.acquire()

    def resume(self):
        if self.paused:
            self.paused = False
            self.pause_cond.notify()
            self.pause_cond.release()

    def stop(self):
        self.running = False
