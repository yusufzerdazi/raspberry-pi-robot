"""Module that contains general functions for 2D algebraic manipulations."""

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
    theta = math.degrees(np.arctan2(coordinates[1], coordinates[0]))
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
    x = r * np.cos(math.radians(theta))
    y = r * np.sin(math.radians(theta))
    return np.array([x, y])


def rotate_point(centre, point, angle):
    """Rotate a point counterclockwise by a given angle around a given centre.

    Args:
        centre (numpy.ndarray): Centre of rotation
        point (numpy.ndarray): Point to rotate
        angle (float): Angle of rotation (in degrees)

    Returns:
        numpy.array: Rotated point
    """
    ox, oy = centre
    px, py = point

    qx = ox + math.cos(math.radians(angle)) * (px - ox) - math.sin(math.radians(angle)) * (py - oy)
    qy = oy + math.sin(math.radians(angle)) * (px - ox) + math.cos(math.radians(angle)) * (py - oy)

    return np.array([qx, qy])


def closest_point(point, slope, intercept):
    """Given a point and a line, find the point on the line closest to the point.

    Args:
        point (numpy.ndarray): Point
        slope (float): Gradient of the line
        intercept (float): y-intercept of the line

    Returns:
        numpy.array: Closest point on line.
    """
    a = slope
    b = -1
    c = intercept

    x = (b*(b*point[0] - a*point[1]) - a*c)/(a**2 + b**2)
    y = (a*(-b*point[0] + a*point[1]) - b*c)/(a**2 + b**2)

    return np.array([x, y])


def line_equation(A, B):
    """Given two points on a line, find its equation.
    
    Args:
        A (numpy.ndarray): First point
        B (numpy.ndarray): Second point

    Returns:
        tuple: slope, intercept pair.
    """
    slope = (A[1] - B[1])/(A[0] - B[0])
    intercept = A[1] - slope*A[0]
    return slope, intercept


def line_intersection(slope1, intercept1, slope2, intercept2):
    """Given two line equations, find their intersection point.

    Args:
        slope1 (float): SLope of first line
        intercept1 (float): y-intercept of first line
        slope2 (float): Slope of second line
        intercept2 (float): y-intercept of second line

    Returns:
        np.array: Their intersection point.
    """
    # If they are parallel, they don't intersect (or their intersection is infinite).
    if slope1 == slope2:
        return -1

    x = (intercecp2-intercept1)/(slope1-slope2)
    y = slope2*x + intercept2

    return np.array([x, y])


def rotate_line(centre, angle, slope, intercept):
    """Rotate a line by a given angle about a given centre.

    Args:
        centre (array): Centre of rotation
        angle (float): Angle to rotate by (in degrees)
        slope (float): Gradient of line
        intercept (float): y-intercept of line.

    Returns:
        tuple: Equation of rotated line.
    """
    A = np.array([0, intercept])
    B = np.array([1, slope + intercept])

    Ar = rotate_point(centre, A, angle)
    Br = rotate_point(centre, B, angle)

    return line_equation(Ar, Br)


def point_distance(A, B):
    """Distance between two points.

    Args:
        A (np.array): First point
        B (np.array): Second point

    Returns:
        float: Distance between the points.
    """
    return np.linalg.norm(A - B)


def line_distance(P, slope, intercept):
    """Distance between point and line.

    Args:
        P (np.array): Point
        slope: Gradient of line
        intercept: y-intercept of line.

    Returns:
        float: Distance between the point and the line
    """
    return point_distance(P, closest_point(P, slope, intercept))


def angle_distance(alpha, beta):
    """Get the smallest angle distance between two angles (i.e. distance between 350 and 10 is 20).

    Args:
        alpha (float): First angle
        beta (float): Second angle

    Returns:
        Distance between the two angles.
    """
    phi = abs(beta - alpha) % 360  # This is either the distance or 360 - distance
    if phi > 180:
        return 360 - phi
    else:
        return phi


def normal(gradient):
    n = np.array([-gradient, 1])
    return n/np.linalg.norm(n)

def _c(ca,i,j,P,Q):
    if ca[i,j] > -1:
        return ca[i,j]
    elif i == 0 and j == 0:
        ca[i,j] = point_distance(P[0],Q[0])
    elif i > 0 and j == 0:
        ca[i,j] = max(_c(ca,i-1,0,P,Q),point_distance(P[i],Q[0]))
    elif i == 0 and j > 0:
        ca[i,j] = max(_c(ca,0,j-1,P,Q),point_distance(P[0],Q[j]))
    elif i > 0 and j > 0:
        ca[i,j] = max(min(_c(ca,i-1,j,P,Q),_c(ca,i-1,j-1,P,Q),_c(ca,i,j-1,P,Q)),point_distance(P[i],Q[j]))
    else:
        ca[i,j] = float("inf")
    return ca[i,j]

""" Computes the discrete frechet distance between two polygonal lines
Algorithm: http://www.kr.tuwien.ac.at/staff/eiter/et-archive/cdtr9464.pdf
P and Q are arrays of 2-element arrays (points)
"""
def frechetDist(P,Q):
    ca = np.ones((len(P),len(Q)))
    ca = np.multiply(ca,-1)
    return _c(ca,len(P)-1,len(Q)-1,P,Q)

import numpy as np

def closestDistanceBetweenLines(P, Q):

    ''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
        Return the closest points on each segment and their distance
    '''
    a0 = np.append(P[0], [0])
    a1 = np.append(P[1], [0])
    b0 = np.append(Q[0], [0])
    b1 = np.append(Q[1], [0])

    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)

    _A = A / magA
    _B = B / magB

    cross = np.cross(_A, _B);
    denom = np.linalg.norm(cross)**2


    # If lines are parallel (denom=0) test if lines overlap.
    # If they don't overlap then there is a closest point solution.
    # If they do overlap, there are infinite closest positions, but there is a closest distance
    if not denom:
        d0 = np.dot(_A,(b0-a0))

        d1 = np.dot(_A,(b1-a0))

        # Is segment B before A?
        if d0 <= 0 >= d1:
            if np.absolute(d0) < np.absolute(d1):
                return a0,b0,np.linalg.norm(a0-b0)
            return a0,b1,np.linalg.norm(a0-b1)


        # Is segment B after A?
        elif d0 >= magA <= d1:
            if np.absolute(d0) < np.absolute(d1):
                return a1,b0,np.linalg.norm(a1-b0)
            return a1,b1,np.linalg.norm(a1-b1)


        # Segments overlap, return distance between parallel segments
        return None,None,np.linalg.norm(((d0*_A)+a0)-b0)



    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0);
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA/denom;
    t1 = detB/denom;

    pA = a0 + (_A * t0) # Projected closest point on segment A
    pB = b0 + (_B * t1) # Projected closest point on segment B


    # Clamp projections
    if t0 < 0:
        pA = a0
    elif t0 > magA:
        pA = a1

    if t1 < 0:
        pB = b0
    elif t1 > magB:
        pB = b1

    # Clamp projection A
    if (t0 < 0) or (t0 > magA):
        dot = np.dot(_B,(pA-b0))
        if dot < 0:
            dot = 0
        elif dot > magB:
            dot = magB
        pB = b0 + (_B * dot)

    # Clamp projection B
    if (t1 < 0) or (t1 > magB):
        dot = np.dot(_A,(pB-a0))
        if dot < 0:
            dot = 0
        elif dot > magA:
            dot = magA
        pA = a0 + (_A * dot)


    return pA,pB,np.linalg.norm(pA-pB)