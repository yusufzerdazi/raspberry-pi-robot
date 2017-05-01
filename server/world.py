"""Module defining functions for extracting and matching landmarks."""

import numpy
from scipy.stats import stats
from skimage.transform import probabilistic_hough_line

from server import util, occupancy
from server import metrics


MAX_TRIALS = 1000
ASSIGNED = 0.7
SAMPLE_SIZE = 5
ANGLE_RANGE = 30
RANSAC_TOLERANCE = 5
RANSAC_CONSENSUS = 60
LANDMARK_RADIUS = 20


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
                        new = util.length([x.segment[i], y.segment[j]])
                        if new > current:
                            current = new
                            segment = [x.segment[i], y.segment[j]]
        return Landmark(segment, self.association)

    def transform(self, centre, angle=0.0, position=numpy.array([0, 0])):
        translated = [self.segment[0] + position, self.segment[1] + position]
        rotated = util.rotate_points(centre, translated, angle)
        return Landmark(rotated, self.association)

    def distance(self, other):
        return metrics.origin_distance(self.segment, other.segment)

    def probability(self, other):
        return 1/((self.distance(other)+1)/10)


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


def limit_landmarks(landmarks):
    new = []
    if len(landmarks) > 1:
        for l in landmarks:
            add = True
            for j in new:
                if l.distance(j) < LANDMARK_RADIUS:
                    add = False
            if add:
                new.append(l)
    return new


def extract_landmarks(measurements):
    """Function which extracts landmarks from a set of points.

    Args:
        measurements (:obj:`list` of :obj:`Measurement`): A set of Measurement objects that detail the robots
            observations.

    Returns:
        list: A list of discovered landmarks.
    """
    landmarks = []  # Array to store detected landmarks.
    assigned = []
    trials = 0  # Number of trials attempted.

    # Look for landmarks while haven't reached max trials, and still enough unassigned points left.
    # noinspection PyTypeChecker
    while trials < MAX_TRIALS and len(assigned) < ASSIGNED * len(measurements):

        # Randomly select a clustered subsample of the points.
        unassigned = [m for m in measurements if m not in assigned]
        angle = numpy.random.choice(unassigned).angle
        radius = [p for p in unassigned if util.angle_diff(p.angle, angle) < ANGLE_RANGE]
        sample = numpy.array([p for p in numpy.random.choice(radius, SAMPLE_SIZE)])

        # Calculate the x and y standard deviation of the sample.
        std_x = numpy.std(numpy.array([p.location for p in sample])[:, 0])
        std_y = numpy.std(numpy.array([p.location for p in sample])[:, 1])
        is_vertical = std_x == 0 or std_y / std_x > 1

        consensus = find_consensus(unassigned, sample, is_vertical)

        # If there are enough matching points, a landmark has been found.
        if len(consensus) > RANSAC_CONSENSUS:
            segment = recalculate_line(consensus, is_vertical)
            if not segment:
                trials += 1
                continue
            assigned.extend(consensus)
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
        if util.point_line_dist(cartesian_unassigned[i], slope, intercept) < RANSAC_TOLERANCE:
            consensus.append(unassigned[i])  # Add it to the consensus points.

    return consensus


def recalculate_line(consensus, is_vertical):
    cartesian_consensus = numpy.array([point.location for point in consensus])
    # If almost vertical, calculate line in terms of y.
    if is_vertical:
        cartesian_consensus = numpy.fliplr(cartesian_consensus)

    # Calculate regression line.
    slope, intercept, r_, p_, e_ = stats.linregress(cartesian_consensus[:, 0], cartesian_consensus[:, 1])

    start = util.nearest(cartesian_consensus[0], slope, intercept)
    end = util.nearest(cartesian_consensus[0], slope, intercept)
    distance = 0

    for i in range(len(consensus)):
        for j in range(i+1, len(consensus)):
            point_a = util.nearest(cartesian_consensus[i], slope, intercept)
            point_b = util.nearest(cartesian_consensus[j], slope, intercept)
            new_dist = util.dist(point_a, point_b)
            if new_dist > distance:
                distance = new_dist
                start = point_a
                end = point_b

    # If line is vertical, flip coordinates back.
    if is_vertical:
        start = numpy.flipud(start)
        end = numpy.flipud(end)

    return start, end


def extract_hough_landmarks(image):
    """Extract landmarks from an image using the Hough transform.
    
    Args:
        image (PIL.Image.Image): Image to extract landmarks from.
    """
    origin = numpy.array(image.size, dtype=numpy.int)/2  # Centre of image.
    black_white = occupancy.black_white(image)
    lines = probabilistic_hough_line(black_white, threshold=30, line_length=50, line_gap=50)  # Extract lines
    landmarks = []
    for line in lines:
        landmarks.append(Landmark([line[0] - origin[0], line[1] - origin[1]]))
    return limit_landmarks(landmarks)
