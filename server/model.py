import random

import numpy
from scipy.stats import stats

from server import algebra
from server import metrics


MAX_TRIALS = 1000
ASSIGNED = 100
SAMPLE_SIZE = 10
ANGLE_RANGE = 30
RANSAC_TOLERANCE = 10
RANSAC_CONSENSUS = 20
LANDMARK_RADIUS = 150


class Landmark(object):
    """

    Attributes:
        segment (list): Line segment.
    """

    def __init__(self, segment, association=None):
        self.association = association
        self.segment = segment

    def __add__(self, other):
        """Merge two landmarks into one, by taking the two endpoints that maximise their length."""
        current = 0
        segment = []
        for x in [self, other]:
            for y in [self, other]:
                for i in range(2):
                    for j in range(2):
                        new = algebra.length([x.segment[i], y.segment[j]])
                        if new > current:
                            current = new
                            segment = [x.segment[i], y.segment[j]]
        return Landmark(segment, self.association)

    def transform(self, centre, angle=0.0, position=numpy.array([0, 0])):
        return Landmark(algebra.rotate_points(centre, [self.segment[0] + position, self.segment[1] + position], angle),
                        self.association)

    def distance(self, other):
        return metrics.frechet_distance(self.segment, other.segment)


def associate_landmarks(new_landmarks, landmarks):
    for l in new_landmarks:
        # Initialise variables
        distance = 99999
        closest = None

        # Find the closest known landmark.
        for j in landmarks:
            dist = l.distance(j)
            if dist < distance:
                distance = dist
                closest = j

        # If the closest known landmark is within a certain radius, assume they are the same.
        if distance < LANDMARK_RADIUS:
            l.association = closest


def extract_landmarks(measurements):
    """Function which extracts landmarks from a set of points.

    Args:
        measurements (:obj:`list` of :obj:`Measurement`): A set of Measurement objects that detail the robots observations.

    Returns:
        list: A list of discovered landmarks.
    """
    landmarks = []  # Array to store detected landmarks.
    assigned = []
    trials = 0  # Number of trials attempted.

    # Look for landmarks while haven't reached max trials, and still enough unassigned points left.
    while trials < MAX_TRIALS and len(assigned) < ASSIGNED:

        # Randomly select a clustered subsample of the points.
        unassigned = [m for m in measurements if m not in assigned]
        angle = numpy.random.choice(unassigned).angle
        radius = [p for p in unassigned if algebra.angle_diff(p.angle, angle) < ANGLE_RANGE]
        sample = numpy.array([p for p in numpy.random.choice(radius, 3)])

        # Calculate the x and y standard deviation of the sample.
        std_x = numpy.std(numpy.array([p.location for p in sample])[:, 0])
        std_y = numpy.std(numpy.array([p.location for p in sample])[:, 1])
        is_vertical = std_x == 0 or std_y / std_x > 1

        consensus = find_consensus(unassigned, sample, is_vertical)

        # If there are enough matching points, a landmark has been found.
        if len(consensus) > RANSAC_CONSENSUS:
            segment = recalculate_line(consensus, is_vertical)
            landmarks.append(Landmark(segment))
            trials = 0  # Found a landmark, so reset trials.
        else:
            trials += 1
    return landmarks


def find_consensus(unassigned, sample, is_vertical):
    cartesian_sample = numpy.array([point.location for point in sample])
    cartesian_unassigned = numpy.array([point.location for point in unassigned])
    consensus = []

    # If almost vertical, calculate line in terms of y.
    if is_vertical:
        cartesian_sample = numpy.fliplr(cartesian_sample)
        cartesian_unassigned = numpy.fliplr(cartesian_unassigned)

    # Calculate regression line.
    slope, intercept, r_, p_, e_ = stats.linregress(cartesian_sample[:, 0], cartesian_sample[:, 1])

    # Find the unassigned points that match to this line.
    for i in range(len(unassigned)):
        # If the point lies close enough to the line.
        if algebra.point_line_dist(cartesian_unassigned[i], slope, intercept) < RANSAC_TOLERANCE:
            consensus.append(unassigned[i])  # Add it to the consensus points.

    return consensus


def recalculate_line(consensus, is_vertical):
    consensus.sort(key=lambda x: x.angle)
    cartesian_consensus = numpy.array([point.location for point in consensus])
    # If almost vertical, calculate line in terms of y.
    if is_vertical:
        cartesian_consensus = numpy.fliplr(cartesian_consensus)

    # Calculate regression line.
    slope, intercept, r_, p_, e_ = stats.linregress(cartesian_consensus[:, 0], cartesian_consensus[:, 1])

    angle_range = 0
    start = None
    end = None
    for i in range(len(consensus)):
        angle = algebra.angle_diff(consensus[i].angle, consensus[i - 1].angle)
        if angle > angle_range:
            start = algebra.nearest(cartesian_consensus[i], slope, intercept)
            end = algebra.nearest(cartesian_consensus[i-1], slope, intercept)
            angle_range = angle

    if is_vertical:
        start = numpy.flipud(start)
        end = numpy.flipud(start)

    return start, end
