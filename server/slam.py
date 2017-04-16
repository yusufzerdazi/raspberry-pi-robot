import cProfile
import threading
import time
import copy

import numpy
from PIL import Image
from scipy import stats
from server import metrics
from server.robot import Measurement
from server import util
from server import model

SAMPLES = 400

DISTANCE_MEAN = 100.1188119
DISTANCE_STDDEV = 8.803385712

ANGLE_MEAN = 90.01
ANGLE_STDDEV = 4.07797744


class Slam(threading.Thread):
    """Class which contains the main SLAM loop.
    
    Attributes:
        running (bool): Thread running.
        landmarks (array): List of unique landmarks.
        prev (robot.State): Copy of previous robot state.
        paused (bool): SLAM paused.
        pause_cond (threading.Condition): Condition for managing paused state.
    """
    def __init__(self, comm, grid):
        """Initialise SLAM object.

        Args:
            robot (robot.Robot): Robot object.
            occupancy (robot.Map): Occupancy grid.
            controlled (bool): Whether to run SLAM automatically or wait for user input.
        """
        threading.Thread.__init__(self)

        self.comm = comm
        self.grid = grid
        self.running = True
        self.controlled = False
        self.allow_control = self.controlled
        self.landmarks = []
        self.prev = copy.deepcopy(self.comm.robot)
        self.current = copy.deepcopy(self.comm.robot)

        self.landmark_mode = util.LandmarkMode.HOUGH
        self.slam_mode = util.SlamMode.LANDMARKS

        self.paused = False
        self.pause_cond = threading.Condition(threading.Lock())

    def run(self):
        i = 1
        while i <= 300:
            #l1 = model.Landmark([numpy.array([50,-50]), numpy.array([50,50])])
            #l12 = model.Landmark([numpy.array([-50, 50]), numpy.array([50, 50])])
            #self.landmarks.extend([l1,l12])
            #l2 = model.Landmark([numpy.array([55,-50]), numpy.array([50,50])])
            #l22 = model.Landmark([numpy.array([-50, 45]), numpy.array([50, 50])])
            #l2.association = l1
            #l22.association = l12
            #dist = self.get_landmark_distribution([l2, l22],[0], [(i,j) for i in range(-10,10) for j in range(-10,10)])
            #plot_dist = {}
            #for key in dist:
            #    plot_dist[key[0]] = plot_dist.get(key[0], 0) + dist[key]
            #self.grid.plot_prob_dist(plot_dist,self.grid.probability_mode.SLAM_PROBABILITIES)
            #self.landmarks.extend([l2, l22])
            #while True:
            #    time.sleep(.02)
            self.wait_for_command()
            measurements = self.take_measurements()
            self.update_model(measurements)
            self.move_robot(measurements)

            print(str(i) + "," + str(time.time()) + "," + str(util.dist(self.comm.actual, self.comm.robot.adjusted.location)) + "," + str(util.dist(self.comm.actual, self.comm.robot.state.location)))
            i += 1

    def get_scan_matching_distribution(self, angles, translations, centre):
        dist = {}
        global_map = self.grid.black_white(self.grid.view_images[self.grid.view_mode.ADJUSTED])
        local_map = self.grid.black_white(self.grid.view_images[self.grid.view_mode.LOCAL])
        for translation in translations:
            translated_map = self.grid.translate(local_map, 0, centre, centre+translation)
            for angle in angles:
                rotated_map = self.grid.translate(translated_map, angle, centre+translation)
                rotated_map.paste(global_map, mask=rotated_map)
                value = numpy.array(rotated_map).mean()
                dist[(tuple(translation),  angle)] = value
        return dist

    def get_landmark_distribution(self, landmarks, angles, translations):
        landmark_distribution = {}
        for dp in translations:  # For each alternative distance.
            for ap in angles:  # For each alternative angle turned.

                # Calculate the position if this angle and distance had been moved.
                alternative_location = numpy.array(dp + self.current.adjusted.location)

                landmark_distribution[(tuple(dp), ap)] = 1.0

                for landmark in landmarks:
                    # Find where this landmark would be if the robot had moved to this alternate location.
                    alternative_landmark = landmark.transform(alternative_location, ap, dp)

                    # Find probability of this matching its associated landmark, based on inverse distance.
                    landmark_distribution[(tuple(dp), ap)] *= alternative_landmark.probability(landmark.association)

        return landmark_distribution
    
    def get_prior_distribution(self):
        # Calculate change in angle and distance moved.
        turned = self.current.adjusted.heading - self.prev.adjusted.heading
        distance = util.dist(self.current.adjusted.location, self.prev.adjusted.location)

        # Calculate probability distribution for angle turned.
        angle_std = ((abs(turned)) / ANGLE_MEAN) * ANGLE_STDDEV + 5
        angle_keys = [i for i in range(int(-angle_std * 2), int(angle_std * 2) + 1)]
        angle_values = stats.norm.pdf(angle_keys, 0, angle_std)
        angle_probs = {angle_keys[i]: angle_values[i] for i in range(len(angle_keys))}

        # Calculate probability distribution for position.
        distance_std = (distance / DISTANCE_MEAN) * DISTANCE_STDDEV + 1
        distance_distribution = stats.norm(0, distance_std)

        position_keys = [numpy.array([i, j]) for i in range(-3, 4) for j in range(-3, 4)]
        prior_distribution = {}
        for pos in position_keys:
            for ang in angle_keys:
                distance = util.dist(self.current.adjusted.location, pos+self.current.adjusted.location)
                prior_distribution[tuple(pos), ang] = distance_distribution.pdf(distance) * angle_probs[ang]
                
        return prior_distribution, angle_keys, position_keys

    def get_hough_landmarks(self):
        local_map = self.grid.black_white(self.grid.view_images[self.grid.view_mode.LOCAL])
        segments = self.grid.detect_lines(local_map)
        landmarks = [model.Landmark(segment) for segment in segments]
        return landmarks

    def get_ransac_landmarks(self):
        pass

    def update_model(self, measurements):
        prior_distribution, angles, translations = self.get_prior_distribution()
        landmarks = []

        if self.slam_mode == util.SlamMode.LANDMARKS:
            if self.landmark_mode == util.LandmarkMode.HOUGH:
                landmarks = model.limit_landmarks(self.get_hough_landmarks())
            else:
                landmarks = model.extract_landmarks(measurements)
            model.associate_landmarks(landmarks, self.landmarks)
            associated_landmarks = [l for l in landmarks if l.association]
            slam_distribution = self.get_landmark_distribution(associated_landmarks, angles, translations)
        else:
            slam_distribution = self.get_scan_matching_distribution(angles, translations, self.current.adjusted.location)
            smallest = min(list(slam_distribution.values()))
            slam_distribution = {key: slam_distribution[key] - smallest for key in slam_distribution}

        normalised_slam_distribution = util.normalise_distribution(slam_distribution)
        normalised_prior_distribution = util.normalise_distribution(prior_distribution)
        combined_distribution = {key: normalised_prior_distribution[key] * normalised_slam_distribution[key]
                                 for key in normalised_prior_distribution}

        self.plot_distributions(normalised_prior_distribution, normalised_slam_distribution, combined_distribution)

        delta = max(combined_distribution, key=combined_distribution.get)
        self.comm.robot.adjustment.delta(delta[0], delta[1])
        adjusted_map = self.grid.translate(self.grid.view_images[self.grid.view_mode.LOCAL], -delta[1],
                                           self.current.adjusted.location + self.grid.origin,
                                           self.current.adjusted.location + self.grid.origin + numpy.array(delta[0]))

        global_map = numpy.array(self.grid.view_images[self.grid.view_mode.ADJUSTED].convert("L")).astype(float)
        local_map = numpy.array(adjusted_map.convert("L")).astype(float)
        self.grid.view_images[self.grid.view_mode.ADJUSTED] = Image.fromarray(global_map*local_map*2/255).convert("RGBA")

        adjusted_landmarks = [landmark.transform(self.current.adjusted.location, delta[1], delta[0]) for landmark in
                              landmarks if not landmark.association]
        self.landmarks.extend(adjusted_landmarks)

    def plot_distributions(self, normalised_prior_distribution, normalised_slam_distribution, combined_distribution):
        plot_prior_distribution = {}
        plot_slam_distribution = {}
        plot_combined_distribution = {}
        loc = self.current.adjusted.location.astype(int)
        for key in normalised_slam_distribution:
            new_key = tuple(numpy.array(key[0]) + loc)
            plot_slam_distribution[new_key] = plot_slam_distribution.get(new_key, 0) + normalised_slam_distribution[key]
            plot_combined_distribution[new_key] = plot_combined_distribution.get(new_key, 0) + combined_distribution[key]
            plot_prior_distribution[new_key] = plot_prior_distribution.get(new_key, 0) + normalised_prior_distribution[key]

        self.grid.plot_prob_dist(plot_slam_distribution, self.grid.probability_mode.SLAM_PROBABILITIES)
        self.grid.plot_prob_dist(plot_prior_distribution, self.grid.probability_mode.PRIOR_PROBABILITIES)
        self.grid.plot_prob_dist(plot_combined_distribution, self.grid.probability_mode.COMBINED_PROBABILITIES)
        
    def wait_for_command(self):
        if self.controlled:
            self.pause()
        with self.pause_cond:
            while self.paused:
                self.pause_cond.wait()
        self.allow_control = False
        self.comm.move(0, False)
        self.current = copy.deepcopy(self.comm.robot)

    def take_measurements(self):
        self.grid.clear()
        self.comm.get_measurements()
        while len(self.comm.measurements) < SAMPLES:
            time.sleep(.02)

        measurements = self.comm.get_median_measurements()
        result = []
        for measurement in measurements:
            if measurement.distance < 255:
                result.append(measurement)
                self.grid.plot_measurement(measurement)

        return result

    def move_robot(self, measurements):
        self.prev = copy.deepcopy(self.current)
        self.allow_control = self.controlled
        if not self.controlled:
            obstacles = [m for m in measurements if util.angle_diff(m.angle, 0) < 20 and m.distance < 30]
            if len(obstacles) > 0:
                self.comm.turn(90)
            self.comm.drive(30)

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
