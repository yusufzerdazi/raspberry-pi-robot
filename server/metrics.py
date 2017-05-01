"""Module which defines the functions used to measure the distance between two landmarks."""

import math
import numpy

from server.util import dist
from server.util import length


def frechet_distance(first, second):
    """Since the Frechet distance is dependant on the order that the line segments are provided, find the minimum.
    
    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Frechet distance.
    """
    return min(ordered_frechet_distance(first, second), ordered_frechet_distance(second, first))


def ordered_frechet_distance(first, second):
    """Computes the discrete frechet distance between two polygonal lines
    Algorithm: http://www.kr.tuwien.ac.at/staff/eiter/et-archive/cdtr9464.pdf
    P and Q are arrays of 2-element arrays (points)

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Frechet distance.
    """
    a = dist(first[0], second[0])
    b = max(dist(first[0], second[1]), a)
    c = max(dist(first[1], second[0]), a)
    d = max(min(a, b, c), dist(first[1], second[1]))

    return d


def hausdorff_distance(first, second):
    """Computes the Hausdorff distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Hausdorff distance.
    """
    a = point_segment_distance(first[0], second)
    b = point_segment_distance(first[1], second)
    c = point_segment_distance(second[0], first)
    d = point_segment_distance(second[1], first)

    return max(max(a, b), max(c, d))


def straight_line_distance(first, second):
    """Computes the Straight Line distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Straight line distance.
    """
    a = dist(first[0], second[0])
    b = dist(first[0], second[1])
    c = dist(first[1], second[0])
    d = dist(first[1], second[1])

    distances = []
    for s, t in [[first, second], [second, first]]:
        distance = (a + b + c + d)/4
        distance += (length(s) + length(t)) / 4
        distance -= mod_hausdorff_distance(s, t) / 4
        distance += perpendicular_distance(s, t)

        distances.append(distance)

    return min(distances)


def min_distance(first, second):
    """Computes the minimum distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Minimum distance.
    """
    if not segments_intersect(first, second) is None:
        return 0.0
    distances = []
    for s, t in [[first, second], [second, first]]:
        for i in [0, 1]:
            distances.append(point_segment_distance(s[i], t))
    return min(distances)


def perpendicular_distance(first, second):
    """Computes the perpendicular distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Perpendicular distance.
    """
    if not segments_intersect(first, second) is None:
        return 0.0
    distances = []
    for s, t in [[first, second], [second, first]]:
        for i in [0, 1]:
            distances.append(point_perpendicular_distance(s[i], t))
    return min(distances)


def mod_hausdorff_distance(first, second):
    """Computes the Modified Hausdorff distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Modified Hausdorff distance.
    """
    return min(length(first), length(second)) * math.sin(angle(first, second))


# noinspection PyTypeChecker
def midpoint_distance(first, second):
    """Computes the Midpoint distance between two line segments.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Modified Hausdorff distance.
    """
    first_mid = (first[0] + first[1]) / 2
    second_mid = (second[0] + second[1]) / 2

    return min(dist(first[0], second[0]) + dist(first[1], second[1]),
               dist(first[0], second[1]) + dist(first[1], second[0])) + 3 * dist(first_mid, second_mid)


def origin_distance(first, second):
    """Computes the distance between the closest point on each lines to the origin.

    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Distance.
    """
    first_origin = perpendicular_point(numpy.zeros(2), first)
    second_origin = perpendicular_point(numpy.zeros(2), second)
    return dist(first_origin, second_origin)


def segments_intersect(first, second):
    """Determine whether two segments in the plane intersect.
    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        bool: Whether they intersect.
    """
    dx1 = first[1][0] - first[0][0]
    dy1 = first[1][1] - first[0][1]
    dx2 = second[1][0] - second[0][0]
    dy2 = second[1][1] - second[0][1]
    delta = dx2 * dy1 - dy2 * dx1
    if delta == 0:
        if numpy.any([numpy.array_equal(first[i], second[j]) for i in range(2) for j in range(2)]):
            return None
        return None  # parallel segments
    s = (dx1 * (second[0][1] - first[0][1]) + dy1 * (first[0][0] - second[0][0])) / delta
    t = (dx2 * (first[0][1] - second[0][1]) + dy2 * (second[0][0] - first[0][0])) / (-delta)
    if s < 0 or s > 1 or t < 0 or t > 1:
        return None
    else:
        return numpy.array([first[0][0] + t * dx1, first[0][1] + t * dy1])


def point_segment_distance(point, segment):
    """Shortest distance between a point and a segment.
    Args:
        point (np.ndarray): Point
        segment (list): Segment

    Returns:
        bool: Shortest distance.
    """
    dx = segment[1][0] - segment[0][0]
    dy = segment[1][1] - segment[0][1]
    if dx == dy == 0:  # the segment's just a point
        return math.hypot(point[0] - segment[0][0], point[1] - segment[0][1])

    # Calculate the t that minimizes the distance.
    t = ((point[0] - segment[0][0]) * dx + (point[1] - segment[0][1]) * dy) / (dx * dx + dy * dy)

    # See if this represents one of the segment's
    # end points or a point in the middle.
    if t < 0:
        return dist(point, segment[0])
    elif t > 1:
        return dist(point, segment[1])
    else:
        return point_perpendicular_distance(point, segment)


def point_perpendicular_distance(point, segment):
    """Shortest perpendicular distance between a point and a segment.
    Args:
        point (np.ndarray): Point
        segment (list): Segment

    Returns:
        bool: Shortest perpendicular distance.
    """
    dx = segment[1][0] - segment[0][0]
    dy = segment[1][1] - segment[0][1]
    if dx == dy == 0:  # the segment's just a point
        return math.hypot(point[0] - segment[0][0], point[1] - segment[0][1])

    # Calculate the t that minimizes the distance.
    t = ((point[0] - segment[0][0]) * dx + (point[1] - segment[0][1]) * dy) / (dx * dx + dy * dy)

    near_x = segment[0][0] + t * dx
    near_y = segment[0][1] + t * dy
    dx = point[0] - near_x
    dy = point[1] - near_y

    return math.hypot(dx, dy)


def perpendicular_point(point, segment):
    """Find point on line closest to point.
    Args:
        point (np.ndarray): Point
        segment (list): Segment

    Returns:
        np.ndarray: Closes point.
    """
    dx = segment[1][0] - segment[0][0]
    dy = segment[1][1] - segment[0][1]
    if dx == dy == 0:  # the segment's just a point
        return math.hypot(point[0] - segment[0][0], point[1] - segment[0][1])

    # Calculate the t that minimizes the distance.
    t = ((point[0] - segment[0][0]) * dx + (point[1] - segment[0][1]) * dy) / (dx * dx + dy * dy)

    near_x = segment[0][0] + t * dx
    near_y = segment[0][1] + t * dy

    return numpy.array([near_x, near_y])


def angle(first, second):
    """Find angle between two line segments.
    Args:
        first (list): First segment
        second (list): Second segment

    Returns:
        float: Angle in degrees.
    """
    # Get nicer vector form
    v_first = numpy.array([(first[0][0] - first[1][0]), (first[0][1] - first[1][1])])
    v_second = numpy.array([(second[0][0] - second[1][0]), (second[0][1] - second[1][1])])

    theta = numpy.arctan2((v_first[0]*v_second[1])-(v_first[1]*v_second[0]),
                          (v_first[0]*v_second[0])+(v_first[1]*v_second[1]))
    if abs(theta) > math.pi / 2:
        theta -= numpy.sign(theta)*math.pi
    return theta
