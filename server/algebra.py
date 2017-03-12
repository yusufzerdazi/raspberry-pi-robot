import math

import numpy as np


def rotate_point(centre, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given centre.

    The angle should be given in degrees.
    """
    ox, oy = centre
    px, py = point

    qx = ox + math.cos(math.radians(angle)) * (px - ox) - math.sin(math.radians(angle)) * (py - oy)
    qy = oy + math.sin(math.radians(angle)) * (px - ox) + math.cos(math.radians(angle)) * (py - oy)

    return np.array([qx, qy])


def closest_point(point, slope, intercept):
    a = slope
    b = -1
    c = intercept

    x = (b*(b*point[0] - a*point[1]) - a*c)/(a**2 + b**2)
    y = (a*(-b*point[0] + a*point[1]) - b*c)/(a**2 + b**2)

    return np.array([x, y])


def line_equation(A, B):
    a = (A[1] - B[1])/(A[0] - B[0])
    b = A[1] - a*A[0]
    return a, b


def line_intersection(a1, b1, a2, b2):
    x = (b2-b1)/(a1-a2)
    y = a2*x + b2

    return np.array([x, y])


def rotate_line(centre, angle, a, b):
    A = np.array([0, b])
    B = np.array([1, a + b])

    Ar = rotate_point(centre, A, angle)
    Br = rotate_point(centre, B, angle)

    return line_equation(Ar, Br)


def point_distance(A, B):
    return np.linalg.norm(A - B)


def line_distance(P, slope, intercept):
    return np.linalg.norm(P - closest_point(P, slope, intercept))


def angle_distance(alpha, beta):
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