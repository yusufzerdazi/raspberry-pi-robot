"""Module containing the main SLAM calculations."""

import threading
import time
import copy

import numpy
from PIL import Image
from scipy import stats

from server import util, robot, world, occupancy

SAMPLES = 400

DISTANCE_MEAN = 100.1188119
DISTANCE_STDDEV = 8.803385712

ANGLE_MEAN = 90.01
ANGLE_STDDEV = 4.07797744


class Slam(threading.Thread):
    """Class which contains the main SLAM loop.
    
    Attributes:
        running (bool): Thread running.
        naive (bool): Whether to run naively.
        controlled (bool): Whether the robot is controlled manually.
        comm (communication.Comm): Object for communicating with the Raspberry Pi.
        grid (occupancy.Grid): Object for storing the map.
        allow_control (bool): Whether input commands will be sent to the robot.
        landmarks (array): List of unique landmarks.
        prev (robot.State): Copy of previous robot state.
        current (robot.State): The current state of the robot.
        landmark_mode (util.LandmarkMode): Which landmark extraction method to use.
        slam_mode (util.SlamMode): Which SLAM method to use.
        drift_error (float): Speed to drift in the +x direction.
        paused (bool): SLAM paused.
        pause_cond (threading.Condition): Condition for managing paused state.
    """
    def __init__(self, comm, grid, landmark_type=util.LandmarkMode.HOUGH, drift_error=0):
        """Initialise SLAM object.

        Args:
            comm (robot.Robot): Communication object.
            grid (robot.Grid): Occupancy grid.
            landmark_type (util.LandmarkMode): Method of landmark extraction.
            drift_error (float): Speed to drift in the +x direction.
        """
        threading.Thread.__init__(self)

        self.running = True
        self.controlled = True
        self.naive = False
        self.comm = comm
        self.grid = grid
        self.allow_control = self.controlled
        self.landmarks = []
        self.prev = copy.deepcopy(self.comm.robot)
        self.current = copy.deepcopy(self.comm.robot)

        self.landmark_mode = landmark_type
        self.slam_mode = util.SlamMode.LANDMARKS

        self.comm.drift_error = drift_error

        self.paused = False
        self.pause_cond = threading.Condition(threading.Lock())

    def run(self):
        """Main loop."""
        while self.running:
            self.wait_for_command()
            measurements = self.take_measurements()
            self.update_model(measurements)
            self.move_robot(measurements)

    def get_scan_matching_distribution(self, angles, translations, centre):
        """Get the probability distribution by performing scan matching between the current scan and the known map.
        
        Args:
            angles (list): The angles to search through.
            translations (list): The translations to search through.
            centre (np.ndarray): Coordinates to centre the distribution at.
        """
        dist = {}

        # Convert the maps to black and white
        global_map = occupancy.black_white(self.grid.view_images[self.grid.view_mode.ADJUSTED])
        local_map = occupancy.black_white(self.grid.view_images[self.grid.view_mode.LOCAL])
        for translation in translations:
            translated_map = occupancy.translate(local_map, 0, centre, centre + translation)
            for angle in angles:
                rotated_map = occupancy.translate(translated_map, angle, centre + translation)
                # Find image which is the two overlapped.
                rotated_map.paste(global_map, mask=rotated_map)
                # Get number of places they overlap.
                value = numpy.array(rotated_map).mean()
                dist[(tuple(translation),  angle)] = value
        return dist

    def get_landmark_distribution(self, landmarks, angles, translations):
        """Get the probability distribution by comparing new landmarks to old landmarks.
        
        Args:
            landmarks: The list of new, matched landmarks.
            angles (list): The angles to search through.
            translations (list): The translations to search through.
        """
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
        """Get the prior normal distribution for position and angle."""
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

    def update_model(self, measurements):
        """Update the map based on new measurements, depending on the mode selected.
        
        Args:
            measurements (list): Newly observed measurements.
        """
        # Get the prior probability distribution.
        prior_distribution, angles, translations = self.get_prior_distribution()
        landmarks = []

        if self.slam_mode == util.SlamMode.LANDMARKS:
            # Extract the landmarks.
            if self.landmark_mode == util.LandmarkMode.HOUGH:
                landmarks = world.extract_hough_landmarks(self.grid.view_images[self.grid.view_mode.LOCAL])
            else:
                landmarks = world.extract_landmarks(measurements)
            # Associate the landmarks.
            world.associate_landmarks(landmarks, [l for l in self.landmarks if not l.association])
            associated_landmarks = [l for l in landmarks if l.association]
            # Calculate the SLAM distribution from the extracted landmarks.
            slam_distribution = self.get_landmark_distribution(associated_landmarks, angles, translations)
        else:
            # Calculate the SLAM distribution using scan matching.
            slam_distribution = self.get_scan_matching_distribution(angles, translations,
                                                                    self.current.adjusted.location)
            # Make the minimum value have probability 0.
            smallest = min(list(slam_distribution.values()))
            slam_distribution = {key: slam_distribution[key] - smallest for key in slam_distribution}

        # Normalise the distributions.
        normalised_slam_distribution = util.normalise_distribution(slam_distribution)
        normalised_prior_distribution = util.normalise_distribution(prior_distribution)

        # Combine the distributions by multiplying them.
        combined_distribution = {key: normalised_prior_distribution[key] * normalised_slam_distribution[key]
                                 for key in normalised_prior_distribution}

        # Plot the distributions.
        self.plot_distributions(normalised_prior_distribution, normalised_slam_distribution, combined_distribution)

        # Calculate the optimal amount to move the robot.
        delta = max(combined_distribution, key=combined_distribution.get)
        self.comm.robot.adjustment.delta(delta[0], delta[1])
        adjusted_map = occupancy.translate(self.grid.view_images[self.grid.view_mode.LOCAL], -delta[1],
                                 self.current.adjusted.location + self.grid.origin,
                                 self.current.adjusted.location + self.grid.origin + numpy.array(delta[0]))

        # Combine the latest scan into the image.
        global_map = numpy.array(self.grid.view_images[self.grid.view_mode.ADJUSTED].convert("L")).astype(float)
        local_map = numpy.array(adjusted_map.convert("L")).astype(float)
        self.grid.view_images[self.grid.view_mode.ADJUSTED] = Image.fromarray(global_map * local_map * 2 / 255).convert(
            "RGBA")

        # Adjust the new landmarks and append them to the landmark array.
        adjusted_landmarks = [landmark.transform(self.current.adjusted.location, delta[1], delta[0]) for landmark in
                              landmarks]
        self.landmarks.extend(adjusted_landmarks)

    def plot_distributions(self, normalised_prior_distribution, normalised_slam_distribution, combined_distribution):
        """Plot each of the distributions for visualisation."""
        plot_prior_distribution = {}
        plot_slam_distribution = {}
        plot_combined_distribution = {}
        loc = self.current.adjusted.location.astype(int)

        # Get the location distribution.
        for key in normalised_slam_distribution:
            new_key = tuple(numpy.array(key[0]) + loc)
            plot_slam_distribution[new_key] = plot_slam_distribution.get(new_key, 0) + normalised_slam_distribution[key]
            plot_combined_distribution[new_key] = plot_combined_distribution.get(new_key, 0) + combined_distribution[
                key]
            plot_prior_distribution[new_key] = plot_prior_distribution.get(new_key, 0) + normalised_prior_distribution[
                key]

        # Plot the distributions
        self.grid.plot_prob_dist(plot_slam_distribution, self.grid.probability_mode.SLAM_PROBABILITIES)
        self.grid.plot_prob_dist(plot_prior_distribution, self.grid.probability_mode.PRIOR_PROBABILITIES)
        self.grid.plot_prob_dist(plot_combined_distribution, self.grid.probability_mode.COMBINED_PROBABILITIES)
        
    def wait_for_command(self):
        """Wait for an input, if in manual control mode."""
        if self.controlled and not self.naive:
            self.pause()
        with self.pause_cond:
            while self.paused:
                self.pause_cond.wait()
        if not self.naive:
            self.allow_control = False
            self.comm.move(0, False)
            self.current = copy.deepcopy(self.comm.robot)

    def take_measurements(self):
        """Get measurements."""
        if not self.naive:
            self.grid.clear()
            self.comm.get_measurements()

            while len(self.comm.measurements) < SAMPLES and self.running:
                time.sleep(.02)

            measurements = self.comm.get_median_measurements()
            result = []
            for measurement in measurements:
                if measurement.distance < 255:
                    result.append(measurement)
                    self.grid.plot_measurement(measurement)

            return result
        else:
            for measurement in self.comm.measurements:
                if measurement.distance < 255:
                    self.grid.plot_measurement(measurement, naive=True)

    def move_robot(self, measurements):
        """Move the robot, turning if there is an object in front of it.
        
        Args:
            measurements (list): List of measurements.
        """
        self.prev = copy.deepcopy(self.current)
        self.allow_control = self.controlled
        if not self.controlled:
            obstacles = [m for m in measurements if util.angle_diff(m.angle, 0) < 20 and m.distance < 30]
            if len(obstacles) > 0:
                self.comm.turn(90)
            self.comm.drive(30)

    def pause(self):
        """Pause the SLAM algorithm."""
        if not self.paused:
            self.paused = True
            self.pause_cond.acquire()

    def resume(self):
        """Resume the SLAM algorithm."""
        if self.paused:
            self.paused = False
            self.pause_cond.notify()
            self.pause_cond.release()

    def stop(self):
        """Stop the SLAM algorithm."""
        self.running = False
        self.resume()
