import threading
import time
import copy

import numpy as np
from scipy import stats

from server.robot import Measurement
from server import algebra
from server import model

SAMPLES = 150

SENSOR_MEAN = 100.0148733
SENSOR_STDDEV = 3.685631181

DISTANCE_MEAN = 100.1188119
DISTANCE_STDDEV = 8.803385712

ANGLE_MEAN = 90.01
ANGLE_STDDEV = 4.07797744


def update_robot(prev, current, landmarks, map):
    """

    """
    # Calculate change in angle and distance moved.
    turned = current.heading - prev.heading
    distance = algebra.dist(current.location, prev.location)

    # Calculate probability distribution for angle turned.
    angle_keys = [i for i in range(int(turned-2*ANGLE_STDDEV), int(turned+2*ANGLE_STDDEV))]
    angle_values = stats.norm.pdf(angle_keys, turned, ANGLE_STDDEV)
    angle_probs = {angle_keys[i]: angle_values[i] for i in range(len(angle_keys))}

    # Calculate probability distribution for position.
    dstd = 1 + DISTANCE_STDDEV * (distance/DISTANCE_MEAN)
    position_keys = [np.array([i,j]) for i in range(int(current.location[0]-2*dstd), int(current.location[0]+2*dstd))
                           for j in range(int(current.location[1]-2*dstd), int(current.location[1]+2*dstd))]
    distribution = stats.norm(0, dstd)
    position_probs = {tuple(position_keys[i]): distribution.pdf(algebra.dist(current.location, position_keys[i])) for i in range(len(position_keys))}

    prior_distribution = {}
    slam_distribution = {}
    distances = {}

    for dp in position_probs:  # For each alternative distance.
        for ap in angle_probs:  # For each alternative angle turned.

            # Calculate the position if this angle and distance had been moved.
            alternative_angle = prev.heading+ap
            alternative_location = np.array(dp)

            # Difference between expected location and updated location
            delta_angle = alternative_angle - current.heading
            delta_location = alternative_location - current.location

            prior_distribution[(dp, ap)] = position_probs[dp] * angle_probs[ap]
            distances[(dp, ap)] = distances.get((dp, ap), 0.0)

            prob = 0
            for landmark in landmarks:
                # Find where this landmark would be if the robot had moved to this alternate location.
                alternative_landmark = landmark.transform(alternative_location, delta_angle, delta_location)

                # Find probability of this matching the associated landmark, based on inverse distance.
                distances[(dp, ap)] = distances.get((dp, ap), 0) + alternative_landmark.distance(landmark.association)

                #server.metrics.frechet_distance(alternative_landmark.segment, landmark.segment)

    max_distance = max(list(distances.values())+[0])
    distance_probs = {key: max_distance - distances[key] for key in distances}
    normalised_prior_distribution = algebra.normalise_distribution(prior_distribution)
    normalised_slam_distribution = algebra.normalise_distribution(distance_probs)
    combined_distribution = {key: normalised_prior_distribution[key] * normalised_slam_distribution[key] for key in normalised_prior_distribution}

    prior_loc_distribution = {}
    slam_loc_distribution = {}
    combined_loc_distribution = {}
    for key in normalised_prior_distribution:
        prior_loc_distribution[key[0]] = prior_loc_distribution.get(key[0], 0) + normalised_prior_distribution[key]
        #if key[1] == 0:
        #    print(key)
        #    print(normalised_slam_distribution[key])
        slam_loc_distribution[key[0]] = slam_loc_distribution.get(key[0], 0) + normalised_slam_distribution[key]
        combined_loc_distribution[key[0]] = combined_loc_distribution.get(key[0], 0) + combined_distribution[key]

    map.plot_prob_dist(prior_loc_distribution, map.probability_mode.PRIOR_PROBABILITIES)
    map.plot_prob_dist(slam_loc_distribution, map.probability_mode.SLAM_PROBABILITIES)
    map.plot_prob_dist(combined_loc_distribution, map.probability_mode.PROBABILITIES)

    best = max(combined_distribution, key=combined_distribution.get)
    return np.array(best[0]) - current.location, prev.heading + best[1] - current.heading


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
            measurements = self.collect_measurements()

            # Extract and associate landmarks.
            landmarks = model.extract_landmarks(measurements)
            model.associate_landmarks(landmarks, self.landmarks)

            # Find most probable position based on matched landmarks, and update robot.
            delta = update_robot(self.prev.adjusted, self.robot.adjusted, [l for l in landmarks if l.association], self.map)
            self.robot.adjustment.delta(delta[0], delta[1])

            adjusted_landmarks = [landmark.transform(self.robot.adjusted.location, delta[1]) for landmark in landmarks]

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
                    landmark.association += landmark
                    self.map.plot_landmark(landmark.association, self.robot.adjusted, (255, 0, 0, 255))
                    #if landmark.distance(landmark.association) > 5:
                    #    self.map.plot_landmark(landmark, self.robot.adjusted, (255,0,0,255))

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
        rotate = (len([measurement for measurement in measurements if algebra.angle_diff(0, measurement.angle) < 10 and measurement.distance < 30]) > 0)
        t = time.time()
        self.communication.move(100, rotate)
        while time.time() - t < 2:
            self.communication.sense()
            time.sleep(0.01)
        self.communication.move(0, False)

    def collect_measurements(self, samples=SAMPLES):
        self.communication.resume()  # Start rotating sensor.
        self.robot.reset()  # Reset robot's measurements
        self.map.scanning = True

        # Collect measurements.
        while (len(self.robot.measurements)) < samples:
            time.sleep(.02)
        measurements = list(self.robot.measurements)

        self.map.scanning = False
        self.communication.pause()  # Stop rotating sensor.

        return sorted(measurements, key=lambda x: x.angle)

    def discrete_measurements(self, measurements, windows=30):
        result = []
        width = 360 / windows
        radius = width / 2
        centre = 0

        while centre % 360 == centre:
            distances = [m.distance for m in measurements if algebra.angle_diff(m.angle, centre) < radius]
            if len(distances) > 0:
                measurement = Measurement(self.robot.adjusted, centre, np.median(distances))
                result.append(measurement)
            centre += width

        return result

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
