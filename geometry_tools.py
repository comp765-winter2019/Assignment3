'''
Copyright (c) 2019, Travis Manderson
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import math
import numpy as np

def transform_2d(pose, trans):
    dx = trans[0]
    dy = trans[1]
    dr = trans[2]
    x = pose[0]
    y = pose[1]

    x1 = math.cos(dr)*x + -1*math.sin(dr)*y
    y1 = math.sin(dr)*x + math.cos(dr)*y
    x1 += dx
    y1 += dy
    if len(pose)>2:
        r = pose[2]
        r1 = r + dr
        return [x1, y1, r1]
    else:
        return [x1, y1]

def pi_2_pi(a):
    while a < 0:
        a += 2 * math.pi
    return a % (2 * math.pi)


def small_angle(anglei, anglef):
    """ get the smallest angle between two
        get direction from initial to final """

    # recenter anglei on circle
    anglef -= anglei
    anglef = pi_2_pi(anglef)

    left_dist = anglef
    right_dist = 2*math.pi - anglef

    if left_dist <= right_dist:
        return left_dist, 1
    else:
        return right_dist, -1

def rotation_magnitude(angle_i, angle_f, direction='L'):  # L is counter-clockwise
    if direction == 'L':
        if angle_f > angle_i:
            return (angle_f - angle_i)
        else:
            return (2 * math.pi - (angle_i - angle_f))
    if direction == 'R':
        if angle_i > angle_f:
            return (angle_i - angle_f)
        else:
            return (2 * math.pi - (angle_f - angle_i))
    return 0.0


def arc_length(angle_i, angle_f, radius, direction='L'):  # L is counter-clockwise
    return radius * rotation_magnitude(angle_i, angle_f, direction)


def ccw(A, B, C):
    """ Determine if three points are listed in a counterclockwise order.
    For three points A, B and C. If the slope of the line AB is less than
    the slope of the line AC then the three points are in counterclockwise order.
    See:  http://compgeom.cs.uiuc.edu/~jeffe/teaching/373/notes/x06-sweepline.pdf
    """
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def lines_intersect(A, B, C, D):
    """ do lines AB and CD intersect? """
    i = ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
    return i


def point_of_line_intersect2(s1, e1, s2, e2):
    line1 = [s1, e1]
    line2 = [s2, e2]
    xdiff = (s1[0] - e1[0], s2[0] - e2[0])
    ydiff = (s1[1] - e1[1], s2[1] - e2[1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        return []

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return [(x, y)]

def point_of_line_intersect(s1, e1, s2, e2):
    a1 = e1[1] - s1[1]
    b1 = s1[0] - e1[0]
    c1 = a1 * s1[0] + b1 * s1[1]
    a2 = e2[1] - s2[1]
    b2 = s2[0] - e2[0]
    c2 = a2 * s2[0] + b2 * s2[1]

    det = a1*b2 - a2*b1
    if det == 0:
        return []

    x = (b2*c1 - b1*c2)/det
    y = (a1*c2 - a2 * c1)/det

    if not is_between(x, s2[0], e2[0]):
        return []

    if not is_between(y, s2[1], e2[1]):
        return []

    return [(x, y)]


def is_between(n, a, b):
    return (n - a) * (n - b) <= 0


def line_intersect_circle(center, radius, p1, p2):
    circ_intersections = list()
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    a = dx**2 + dy**2
    b = 2 * (dx * (p1[0] - center[0]) + dy * (p1[1] - center[1]))
    c = (p1[0] - center[0])**2 + (p1[1] - center[1])**2 - radius**2

    discriminant = b**2 - 4 * a * c
    if discriminant <= 0:
        # print("not a secant")
        return circ_intersections

    t1 = (-b + discriminant**0.5) / (2 * a)
    t2 = (-b - discriminant**0.5) / (2 * a)

    intersect1 = (dx * t1 + p1[0], dy * t1 + p1[1])
    intersect2 = (dx * t2 + p1[0], dy * t2 + p1[1])


    between = True
    if is_between(intersect1[0], p1[0], p2[0]) == False:
        between = False
    if is_between(intersect1[1], p1[1], p2[1]) == False:
        between = False
    if between:
        circ_intersections.extend([intersect1])

    between = True
    if is_between(intersect2[0], p1[0], p2[0]) == False:
        between = False
    if is_between(intersect2[1], p1[1], p2[1]) == False:
        between = False
    if between:
        circ_intersections.extend([intersect2])
    # print("circ_intersections: {}, p1: {}, p2: {}".format(circ_intersections, p1, p2))
    return circ_intersections


def line_intersect_arc(center, radius, gamma_i, gamma_f, direction, p1, p2):
    arc_intersections = list()
    if direction == 'R':
        d = 2 * math.pi - gamma_f
        gamma_f = 0
        gamma_i = pi_2_pi(gamma_i + d)
    if direction == 'L':
        d = 2 * math.pi - gamma_i
        gamma_f = pi_2_pi(gamma_f + d)
        gamma_i = 0
    circ_intersections = line_intersect_circle(center, radius, p1, p2)
    if len(circ_intersections) > 0:
        for point in circ_intersections:
            #check if intersections are within gamma_i and gamma_f
            angle = math.atan2((point[1] - center[1]), (point[0] - center[0]))
            angle += d
            angle = pi_2_pi(angle)
            if is_between(angle, gamma_i, gamma_f):
                # if is_between(point[0], p1[0], p1[0]) and is_between(point[1], p1[1], p1[1]):
                arc_intersections.extend([point])
    return arc_intersections


def side_of_line(line0, line1, point):
    x1, y1, x2, y2 = line0[0], line0[1], line1[0], line1[1]
    x, y = point[0], point[1]
    d = (x-x1)*(y2-y1)-(y-y1)*(x2-x1)
    return np.sign(d)


def compare_direction(angle0, angle1):
    angle = angle1 - (angle0 - math.pi/2)
    angle = pi_2_pi(angle)
    if angle <= math.pi:
        return 1
    else:
        return -1


if __name__ == '__main__':
    # a place to debug
    print('debugging geometry tools')
    # print(line_intersect_arc((100,100), 10, 6.1, 0.1, 'L', (0,100),(200,100)))
