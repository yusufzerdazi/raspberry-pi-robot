"""Module that contains general functions for 2D algebraic manipulations."""
import enum
import math
import numpy


def cart_to_pol(coordinates):
    """Convert cartesian coordinates to polar.

    Args:
        coordinates (numpy.ndarray): Coordinates to convert.

    Returns:
        tuple: Polar coordinates.
    """
    r = numpy.linalg.norm(coordinates)
    theta = math.degrees(numpy.arctan2(coordinates[1], coordinates[0]))
    return r, theta


def pol_to_cart(r, theta):
    """
    Convert polar to cartesian coordinates.

    Args:
        r (float): Distance from origin.
        theta (float): Angle.

    Returns:
        numpy.ndarray: Cartesian coordinates.
    """
    x = r * numpy.cos(math.radians(theta))
    y = r * numpy.sin(math.radians(theta))
    return numpy.array([x, y])


def dist(first, second):
    """Distance between two points.

    Args:
        first (numpy.ndarray): First point
        second (numpy.ndarray): Second point

    Returns:
        float: Distance between the points.
    """
    return numpy.linalg.norm(first - second)


def length(segment):
    """Length of a line segment, defined as a list of two points.
    
    Args:
        segment (list): Array of two points.

    Returns:
        float: Length of the segment.
    """
    return dist(segment[0], segment[1])


def angle_diff(alpha, beta):
    """Get the smallest angle distance between two angles (i.e. distance between 350 and 10 is 20).

    Args:
        alpha (float): First angle
        beta (float): Second angle

    Returns:
        Distance between the two angles.
    """
    phi = abs(beta - alpha) % 360  # This is either the distance or 360 - distance
    return 360 - phi if phi > 180 else phi


def rotate_point(centre, point, angle):
    """Rotate a point counterclockwise by a given angle around a given centre.

    Args:
        centre (numpy.ndarray): Centre of rotation
        point (numpy.ndarray): Point to rotate
        angle (float): Angle of rotation (in degrees)

    Returns:
        numpy.ndarray: Rotated point
    """
    ox, oy = centre
    px, py = point

    qx = ox + math.cos(math.radians(angle)) * (px - ox) - math.sin(math.radians(angle)) * (py - oy)
    qy = oy + math.sin(math.radians(angle)) * (px - ox) + math.cos(math.radians(angle)) * (py - oy)

    return numpy.array([qx, qy])


def rotate_points(centre, points, theta):
    """Rotate a list of points counterclockwise by a given angle around a given centre.

    Args:
        centre (numpy.ndarray): Centre of rotation
        points (list): Point to rotate
        theta (float): Angle of rotation (in degrees)

    Returns:
        numpy.ndarray: Rotated point
    """
    return [rotate_point(centre, point, theta) for point in points]


def nearest(point, slope, intercept):
    """Given a point and a line, find the point on the line closest to the point.

    Args:
        point (numpy.ndarray): Point
        slope (float): Gradient of the line
        intercept (float): y-intercept of the line

    Returns:
        numpy.ndarray: Closest point on line.
    """
    a = slope
    b = -1
    c = intercept

    x = (b*(b*point[0] - a*point[1]) - a*c)/(a**2 + b**2)
    y = (a*(-b*point[0] + a*point[1]) - b*c)/(a**2 + b**2)

    return numpy.array([x, y])


def point_line_dist(point, slope, intersect):
    return dist(point, nearest(point, slope, intersect))


def normalise_distribution(d):
    values = list(d.values())

    if len(values) > 0 and sum(values) > 0:
        total = sum(values)
        return {key: d[key] / total for key in d}
    else:
        return {key: 1 / len(d) for key in d}


class TrackingMode(enum.Enum):
    FREE = 0
    STATE = 1
    ADJUSTED = 2


class ViewMode(enum.Enum):
    STATE = 0
    ADJUSTED = 1
    LOCAL = 2


class MapMode(enum.Enum):
    DIST = 0
    PROB = 1
    FINAL = 2


class SlamMode(enum.Enum):
    LANDMARKS = 0
    SCAN_MATCHING = 1


class LandmarkMode(enum.Enum):
    RANSAC = 0
    HOUGH = 1


class ProbabilityMode(enum.Enum):
    COMBINED_PROBABILITIES = 0
    SLAM_PROBABILITIES = 1
    PRIOR_PROBABILITIES = 3
    GLOBAL_MAP = 4
    LOCAL_MAP = 5