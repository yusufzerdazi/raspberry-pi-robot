import threading
import time
import copy

import numpy
from PIL import Image
from scipy import stats
from server import metrics
from server.robot import Measurement
from server import util

SAMPLES = 150

DISTANCE_MEAN = 100.1188119
DISTANCE_STDDEV = 8.803385712

ANGLE_MEAN = 90.01
ANGLE_STDDEV = 4.07797744


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
        self.controlled = True
        self.landmarks = []
        self.prev = copy.deepcopy(self.comm.robot)

        self.landmark_mode = util.LandmarkMode.HOUGH
        self.slam_mode = util.SlamMode.SCAN_MATCHING

        self.paused = False
        self.pause_cond = threading.Condition(threading.Lock())

    def run(self):
        while self.running:
            if self.controlled:
                self.pause()

                # Stop thread running while paused.
            with self.pause_cond:
                while self.paused:
                    self.pause_cond.wait()

                    # If the run was stopped while paused, need to break the loop.
            if not self.running:
                break

            self.current = copy.deepcopy(self.comm.robot)

            # Calculate change in angle and distance moved.
            turned = self.current.adjusted.heading - self.prev.adjusted.heading
            distance = util.dist(self.current.adjusted.location, self.prev.adjusted.location)

            # Calculate probability distribution for angle turned.
            angle_std = ((abs(turned)) / ANGLE_MEAN) * ANGLE_STDDEV + 1
            angle_keys = [i for i in range(int(turned - 2), int(turned + 3))]
            angle_values = stats.norm.pdf(angle_keys, turned, angle_std)
            angle_probs = {angle_keys[i]: angle_values[i] for i in range(len(angle_keys))}

            # Calculate probability distribution for position.
            dstd = (distance / DISTANCE_MEAN) * DISTANCE_STDDEV + 1
            r = min(2 * dstd, 10)
            position_keys = [numpy.array([i, j]) for i in range(int(self.current.adjusted.location[0] - 2),
                                                                int(self.current.adjusted.location[0] + 3))
                             for j in range(int(self.current.adjusted.location[1] - 2),
                                            int(self.current.adjusted.location[1] + 3))]
            distance_distribution = stats.norm(distance, dstd)
            angle_distribution = stats.norm(0, ANGLE_STDDEV)

            position_probs = {tuple(position_keys[i]): angle_distribution.pdf(abs(numpy.degrees(
                metrics.angle([self.prev.adjusted.location, position_keys[i]], [self.prev.adjusted.location,
                                                                                self.current.adjusted.location])))) * distance_distribution.pdf(
                util.dist(position_keys[i], self.prev.adjusted.location)) for i in range(len(position_keys))}

            a = 0
            p = numpy.array([0,0])
            local_map = self.grid.black_white(self.grid.view_images[self.grid.view_mode.LOCAL])
            init_local = self.grid.view_images[self.grid.view_mode.LOCAL].copy()
            self.grid.probability_images[self.grid.probability_mode.GLOBAL_MAP] = self.grid.black_white(self.grid.view_images[self.grid.view_mode.ADJUSTED])
            max_prob = 0
            for dp in position_probs:
                for ap in angle_probs:
                    t = time.time()
                    rotated = self.grid.translate(local_map, ap, self.grid.origin+numpy.array(dp))
                    val = position_probs[dp] * angle_probs[ap] * numpy.sum(numpy.array(rotated.convert("L")) * numpy.array(self.grid.probability_images[self.grid.probability_mode.GLOBAL_MAP].convert("L")))
                    if val > max_prob:
                        max_prob = val
                        a = ap
                        p = numpy.array(dp)
            self.grid.probability_images[self.grid.probability_mode.LOCAL_MAP] = self.grid.translate(local_map, a, self.grid.origin+numpy.array(p))
            loc = numpy.array(self.grid.translate(init_local, a, self.grid.origin+self.comm.robot.adjusted.location).convert("L"))
            glob = numpy.array(self.grid.view_images[self.grid.view_mode.ADJUSTED].convert("L"))
            self.grid.view_images[self.grid.view_mode.ADJUSTED] = Image.fromarray(glob.astype(float)*loc.astype(float)*2/255).convert("RGBA")
            self.comm.robot.adjustment.delta(self.comm.robot.adjusted.location-p, self.comm.robot.adjusted.heading-a)
            self.grid.clear()
            self.prev = copy.deepcopy(self.current)

            if not self.controlled:
                self.comm.drive(50)


    """def run(self):
        ""SLAM Loop""
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

            ""l = model.Landmark([numpy.array([0, 50]), numpy.array([0, -50])])
            self.grid.plot_landmark(l, self.comm.robot.adjusted)
            d1 = {}
            d2 = {}
            d3 = {}
            for i in range(-50, 50):
                for j in range(-50, 50):
                    k = l.transform(numpy.array([0,0]), 0, numpy.array([i, j]))
                    d1[(i,j)] = 1/((metrics.hausdorff_distance(l.segment, k.segment) * k.distance(l))/10+1)
                    d2[(i,j)] = 1/(metrics.hausdorff_distance(l.segment, k.segment)/10+1)
                    d3[(i,j)] = 1/(metrics.origin_distance(l.segment, k.segment)/10+1)
                    self.grid.plot_landmark(k, self.comm.robot.adjusted)
            self.grid.plot_prob_dist(d1, self.grid.probability_mode.SLAM_PROBABILITIES, False)
            self.grid.plot_prob_dist(d2, self.grid.probability_mode.PRIOR_PROBABILITIES, False)
            self.grid.plot_prob_dist(d3, self.grid.probability_mode.COMBINED_PROBABILITIES, False)

            while self.running:
                time.sleep(.02)""

            self.current = copy.deepcopy(self.comm.robot)

            # Collect measurements.
            # measurements = self.discrete_measurements(self.collect_measurements())

            # Extract and associate landmarks.
            #landmarks = model.extract_landmarks(measurements)
            lines = self.grid.detect_lines()
            thingy = []
            landmarks = [model.Landmark([numpy.array(line[0]-self.grid.origin[0]), numpy.array(line[1]-self.grid.origin[1])]) for line in lines]
            for l in landmarks:
                add = True
                for t in thingy:
                    if metrics.min_distance(t.segment, l.segment) < 10:
                        add = False

                if add:
                    thingy.append(l)

            model.associate_landmarks(thingy, self.landmarks)
            # Find most probable position based on matched landmarks, and update robot.
            associated = [l for l in thingy if l.association]

            delta = self.update_robot(associated)
            self.comm.robot.adjustment.delta(delta[0], delta[1])
            self.current.adjustment.delta(delta[0], delta[1])

            adjusted_landmarks = [landmark.transform(self.current.adjusted.location, delta[1], delta[0]) for landmark in thingy]

            # Add/remove landmarks to array.
            self.landmarks.extend(adjusted_landmarks)

            # Set current state to previous state for next round.
            self.prev = copy.deepcopy(self.current)

            # Move robot, if running automatically.
            if not self.controlled:
                self.move()"""

    def update_robot(self, landmarks):
        """

        """
        # Calculate change in angle and distance moved.
        turned = self.current.adjusted.heading - self.prev.adjusted.heading
        distance = util.dist(self.current.adjusted.location, self.prev.adjusted.location)

        # Calculate probability distribution for angle turned.
        angle_std = ((abs(turned))/ANGLE_MEAN) * ANGLE_STDDEV + 1
        angle_keys = [i for i in range(int(turned - 2 * angle_std), int(turned + 2 * angle_std))]
        angle_values = stats.norm.pdf(angle_keys, turned, angle_std)
        angle_probs = {angle_keys[i]: angle_values[i] for i in range(len(angle_keys))}

        # Calculate probability distribution for position.
        dstd = (distance/DISTANCE_MEAN) * DISTANCE_STDDEV + 1
        r = min(2 * dstd, 10)
        position_keys = [numpy.array([i, j]) for i in range(int(self.current.adjusted.location[0] - r), int(self.current.adjusted.location[0] + r + 1))
                         for j in range(int(self.current.adjusted.location[1] - r), int(self.current.adjusted.location[1] + r + 1))]
        distance_distribution = stats.norm(distance, dstd)
        angle_distribution = stats.norm(0, ANGLE_STDDEV)

        position_probs = {tuple(position_keys[i]): angle_distribution.pdf(abs(numpy.degrees(metrics.angle([self.prev.adjusted.location, position_keys[i]], [self.prev.adjusted.location, self.current.adjusted.location])))) * distance_distribution.pdf(util.dist(position_keys[i], self.prev.adjusted.location)) for i in range(len(position_keys))}
        prior_distribution = {}
        slam_distribution = {}

        for dp in position_probs:  # For each alternative distance.
            for ap in angle_probs:  # For each alternative angle turned.

                # Calculate the position if this angle and distance had been moved.
                alternative_angle = self.prev.adjusted.heading + ap
                alternative_location = numpy.array(dp)

                # Difference between expected location and updated location
                delta_angle = alternative_angle - self.current.adjusted.heading
                delta_location = alternative_location - self.current.adjusted.location

                prior_distribution[(dp, ap)] = position_probs[dp] * angle_probs[ap]
                slam_distribution[(dp, ap)] = 1.0

                for landmark in landmarks:
                    # Find where this landmark would be if the robot had moved to this alternate location.
                    alternative_landmark = landmark.transform(alternative_location, delta_angle, delta_location)

                    #assoc = landmark
                    #while assoc:
                    #    assoc = assoc.association

                    #probabilities = [alternative_landmark.probability(assoc) for assoc in hierarchy]

                    # Find probability of this matching its associated landmark, based on inverse distance.
                    slam_distribution[(dp, ap)] *= alternative_landmark.probability(landmark.association)

                    # Find the probability that the new landmark would be observed given its original location
                    # slam_distribution[(dp, ap)] += alternative_landmark.probability(landmark)

        normalised_prior_distribution = util.normalise_distribution(prior_distribution)
        normalised_slam_distribution = util.normalise_distribution(slam_distribution)
        combined_distribution = {key: normalised_prior_distribution[key] * normalised_slam_distribution[key] for key in
                                 normalised_prior_distribution}

        prior_loc_distribution = {}
        slam_loc_distribution = {}
        combined_loc_distribution = {}
        for key in normalised_prior_distribution:
            prior_loc_distribution[key[0]] = prior_loc_distribution.get(key[0], 0.0) + normalised_prior_distribution[
                key]
            slam_loc_distribution[key[0]] = slam_loc_distribution.get(key[0], 0.0) + normalised_slam_distribution[key]
            combined_loc_distribution[key[0]] = combined_loc_distribution.get(key[0], 0.0) + combined_distribution[key]

        self.grid.plot_prob_dist(prior_loc_distribution, self.grid.probability_mode.PRIOR_PROBABILITIES)
        self.grid.plot_prob_dist(slam_loc_distribution, self.grid.probability_mode.SLAM_PROBABILITIES)
        self.grid.plot_prob_dist(combined_loc_distribution, self.grid.probability_mode.COMBINED_PROBABILITIES)

        best = max(combined_distribution, key=combined_distribution.get)
        return numpy.array(best[0]) - self.current.adjusted.location, self.prev.adjusted.heading + best[1] - self.current.adjusted.heading
                
    def move(self, measurements):
        """Move the robot, depending on whether there is an obstacle in front of it.
        
        Args:
            measurements (array): List of measurements.
        """
        # If close enough to object, rotate, otherwise move straight.
        rotate = (len([measurement for measurement in measurements if util.angle_diff(0, measurement.angle) < 10 and measurement.distance < 30]) > 0)
        if rotate:
            self.comm.turn(90)
        else:
            self.comm.drive(20)

    def collect_measurements(self, samples=SAMPLES):
        self.comm.resume()  # Start rotating sensor.
        self.comm.robot.reset()  # Reset robot's measurements
        self.grid.scanning = True

        # Collect measurements.
        while (len(self.comm.robot.measurements)) < samples:
            time.sleep(.02)
        measurements = list(self.comm.robot.measurements)

        self.grid.scanning = False
        self.comm.pause()  # Stop rotating sensor.

        return sorted(measurements, key=lambda x: x.angle)

    def discrete_measurements(self, measurements, windows=30):
        result = []
        width = 360 / windows
        radius = width / 2
        centre = 0

        while centre % 360 == centre:
            distances = [m.distance for m in measurements if util.angle_diff(m.angle, centre) < radius]
            if len(distances) > 0:
                measurement = Measurement(self.comm.robot.adjusted, centre, numpy.median(distances))
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
