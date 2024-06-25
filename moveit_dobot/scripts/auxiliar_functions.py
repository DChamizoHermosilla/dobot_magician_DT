#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy  import pi
import math
from geometry_msgs.msg import Point, Quaternion, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from kinematics_problem import obtain_q1, change_coord, angle2position


l0 = 0.051
l1 = 0.086999874
l2 = 0.134999185
l3 = 0.14700014
l4 = 0.059772341
l5 = 0.091478944
q1_min, q1_max = -pi/2, pi/2
q2_min, q2_max = 0, 85 * pi / 180
q3_min, q3_max = 0, 100 * pi / 180
q4_min, q4_max = -pi/2, pi/2
zmin = 0.013


def validate_joints(joints):
    val = True
    if len(joints) == 4:
        if joints[0] < q1_min or joints[0] > q1_max:
            val = False
        if joints[1] < q2_min or joints[1] > q2_max:
            val = False
        if joints[2] < q3_min or joints[2] > q3_max:
            val = False
        if joints[3] < q4_min or joints[3] > q4_max:
            val = False
    elif len(joints) == 6:
        if joints[0] < q1_min or joints[0] > q1_max:
            val = False
        if joints[1] < q2_min or joints[1] > q2_max:
            val = False
        if joints[3] < q3_min or joints[3] > q3_max:
            val = False
        if joints[5] < q4_min or joints[5] > q4_max:
            val = False
    return val


def validate_position(position):
    return position[2] > zmin


def validate_gripper_limit(distance):
    min_lim = 0
    max_lim = 0.02
    return min_lim <= distance <= max_lim


def moveit_joints_format(joints):
    joints_moveit = np.zeros(6)
    joints_moveit[0], joints_moveit[1], joints_moveit[3], joints_moveit[5] = joints
    joints_moveit[2] = -joints_moveit[1]
    joints_moveit[4] = -joints_moveit[2]
    return joints_moveit


def reduced_joints_format(moveit_joints):
    joints = np.array([moveit_joints[0], moveit_joints[1], moveit_joints[3], moveit_joints[5]])
    return joints


def position_from_pose(pose, l=0.076769716):
    position = [pose.position.x, pose.position.y, pose.position.z]
    position[2] -= l
    
    orientation_quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    orientation = list(euler_from_quaternion(orientation_quat))[2]
    
    return position, orientation


def pose_from_position(position, orientation, l=0.076769716):
    position[2] += l
    p = Point(*position)
    
    q = quaternion_from_euler(*[0, 0, orientation])
    q = Quaternion(*q)
    
    pose = Pose(position = p, orientation = q)
    
    return pose



def obtain_orientation(x, q4):
    q1 = obtain_q1(x)
    if q1 is None:
        return None
    else:
        return q1 + q4



def joint_distance_to_limits(index, joints):
    if index == 1:
        to_min = q1_min - joints[index-1]
        to_max = q1_max - joints[index-1]
    if index == 2:
        to_min = q2_min - joints[index-1]
        if abs((zmin - l0 - l1 - l3 * np.cos(joints[2] + 4/9*pi) + l5) / l2) <= 1:
            q2_zmin = np.arccos((zmin - l0 - l1 - l3 * np.cos(joints[2] + 4/9*pi) + l5) / l2)
            to_max_1 = q2_zmin- joints[index-1]
            to_max_2 = q2_max - joints[index-1]
            to_max = min(to_max_1, to_max_2)
        else: 
            to_max = q2_max - joints[index-1]
    if index == 3:
        to_min = q3_min - joints[index-1]
        if abs((zmin - l0 - l1 - l2 * np.cos(joints[1]) + l5) / l3) <= 1:
            q3_zmin = np.arccos((zmin - l0 - l1 - l2 * np.cos(joints[1]) + l5) / l3) - 4/9*pi
            to_max_1 = q3_zmin- joints[index-1]
            to_max_2 = q3_max - joints[index-1]
            to_max = min(to_max_1, to_max_2)
        else:
            to_max = q3_max - joints[index-1]
    if index == 4:
        to_min = q4_min - joints[index-1]
        to_max = q4_max - joints[index-1]
        
        
    to_max = round(np.degrees(to_max), 3)
    to_min = round(np.degrees(to_min), 3)
        
    return to_min, to_max



def z_max_limit(x):
    q1 = obtain_q1(x)
    x2 = change_coord(x)

    if abs(x2[0]/l3) <= 1:
        q2 = q2_min
        q3 = np.arcsin(x2[0]/l3)
        q3 = pi- q3 - 4*pi/9

    else:
        q2 = np.arcsin((x2[0] - l3 * np.sin(4*pi/9)) / l2)
        q3 = q3_min

    position, _ = angle2position(q1, q2, q3, 0)
    return position[2]



def z_min_limit(x):
    q1 = obtain_q1(x)
    x2 = change_coord(x)

    if abs(x2[0]/l2) <= 1:
        q2 = np.arcsin(x2[0]/l2)
        q3 = q3_max

    else:
        q2 = q2_max
        q3 = np.arcsin((x2[0] - l2 * np.sin(q2)) / l3)
        q3 = pi - q3 - 4*pi/9

    q3_zmin = np.arccos((zmin - l0 - l1 - l2 * np.cos(q2_max) + l5) / l3) - 4*pi/9
    q2_zmin = np.arccos((zmin - l0 - l1 + l3 + l5) / l2)

    if (q2 < q2_zmin) or (q3 < q3_zmin):
        position, _ = angle2position(q1, q2, q3, 0)
        return position[2]

    else:
        return zmin




def x_max_limit(x):
    x2 = change_coord(x)

    if abs((x2[1] - l2 * np.cos(q2_max))/ l3) <= 1:
        q2 = q2_max
        q3 = np.arccos((x2[1] - l2 * np.cos(q2)) / l3) - 4*pi/9
        q1 = np.arcsin(x[1] / (l2 * np.sin(q2) + l3 * np.sin(q3 + 4*pi/9) + l4))

    else:
        q2 = np.arccos((x2[1] - l3 * np.cos(4*pi/9)) / l2)
        q3 = q3_min
        q1 = np.arcsin(x[1] / (l2 * np.sin(q2) + l3 * np.sin(4*pi/9) + l4))

    position, _ = angle2position(q1, q2, q3, 0)
    return position[0]



def x_min_limit(x):
    x2 = change_coord(x)

    if abs((x2[1] - l2)/ l3) <= 1:
        q2 = q2_min
        q3 = np.arccos((x2[1] - l2) / l3) - 4*pi/9
        q1 = np.arcsin(x[1] / (l3 * np.sin(q3 + 4*pi/9) + l4))

    else:
        q2 = np.arccos((x2[1] + l3) / l2)
        q3 = q3_max
        q1 = np.arcsin(x[1] / (l2 * np.sin(q2) - l3 + l4))

    position, _ = angle2position(q1, q2, q3, 0)
    return position[0]


def y_max_limit(x):
    q1 = None
    q2 = None
    q3 = None

    x2 = change_coord(x)

    if abs((x2[1] - l2) / l3) <= 1:
        j2 = q2_min
        j3 = np.arccos((x2[1] - l2) / l3) - 4*pi/9
        if abs(x[0] / (l3 * np.sin(j3 + 4*pi/9) + l4)) <= 1:
            j1 = np.arccos(x[0] / (l3 * np.sin(j3 + 4*pi/9) + l4))
            if (j1 >= q1_min and j1 <= q1_max) and (j2 >= q2_min and j2 <= q2_max) and (j3 >= q3_min and j3 <= q3_max):
                q1 = j1
                q2 = j2
                q3 = j3

    if abs((x2[1] - l3 * np.cos(4*pi/9)) / l2) <= 1:
        j2 = np.arccos((x2[1] - l3 * np.cos(4*pi/9)) / l2)
        j3 = 0
        if abs(x[0] / (l2 * np.sin(j2) + l3 * np.sin(4*pi/9) + l4)) <= 1:
            j1 = np.arccos(x[0] / (l2 * np.sin(j2) + l3 * np.sin(4*pi/9) + l4))
            if (j1 >= q1_min and j1 <= q1_max) and (j2 >= q2_min and j2 <= q2_max) and (j3 >= q3_min and j3 <= q3_max):
                q1 = j1
                q2 = j2
                q3 = j3

    if abs((x2[1] - l2 * np.cos(q2_max)) / l3) <= 1:
        j2 = q2_max
        j3 = np.arccos((x2[1] - l2 * np.cos(j2)) / l3) - 4*pi/9
        if abs(x[0] / (l2 * np.sin(j2) + l3 * np.sin(j3 + 4*pi/9) + l4)) <= 1:
            q1 = np.arccos(x[0] / (l2 * np.sin(j2) + l3 * np.sin(j3 + 4*pi/9) + l4))
            if (j1 >= q1_min and j1 <= q1_max) and (j2 >= q2_min and j2 <= q2_max) and (j3 >= q3_min and j3 <= q3_max):
                q1 = j1
                q2 = j2
                q3 = j3

    if abs((x2[1] + l3) / l2) <= 1:
        j2 = np.arccos((x2[1] + l3) / l2)
        j3 = q3_max
        if abs(x[0] / (l2 * np.sin(j2) - l3 + l4)) <= 1:
            j1 = np.arccos(x[0] / (l2 * np.sin(j2) - l3 + l4))
            if (j1 >= q1_min and j1 <= q1_max) and (j2 >= q2_min and j2 <= q2_max) and (j3 >= q3_min and j3 <= q3_max):
                q1 = j1
                q2 = j2
                q3 = j3


    position, _ = angle2position(q1, q2, q3, 0)
    return position[1]



def y_min_limit(x):
    return -y_max_limit(x)


def cartesian_limits(axis, x):
    if axis == 'x':
        max_lim = x_max_limit(x)
        min_lim = x_min_limit(x)
    if axis == 'y':
        max_lim = y_max_limit(x)
        min_lim = y_min_limit(x)
    if axis == 'z':
        max_lim = z_max_limit(x)
        min_lim = z_min_limit(x)

    return min_lim, max_lim


def cartesian_distance_to_limits(axis, x):
    min_lim, max_lim = cartesian_limits(axis, x)

    if axis == 'x':
        ind = 0
    if axis == 'y':
        ind = 1
    if axis == 'z':
        ind = 2

    distance_to_min = min_lim - x[ind]
    distance_to_max = max_lim - x[ind]

    distance_to_min = round(distance_to_min, 3)
    distance_to_max = round(distance_to_max, 3)

    return distance_to_min, distance_to_max


def orientation_distance_to_limits(x):
    q1= obtain_q1(x)
    if q1 is None:
        return None, None
    else:
        to_max = np.degrees(q1 + q4_max)
        to_min = np.degrees(q1 + q4_min)
        
        to_max = round(to_max, 3)
        to_min = round(to_min, 3)

        return to_min, to_max




def line_points(initial_position, final_position, initial_orientation, final_orientation, step):
    initial_position = np.array(initial_position)
    final_position = np.array(final_position)
    distance = np.linalg.norm(final_position - initial_position)

    num_points = int(distance / step) + 1

    x = np.linspace(initial_position[0], final_position[0], num_points)
    y = np.linspace(initial_position[1], final_position[1], num_points)
    z = np.linspace(initial_position[2], final_position[2], num_points)
    orientation = np.linspace(initial_orientation, final_orientation, num_points)

    points = np.column_stack((x, y, z))

    return points, orientation



def are_points_collinear(p1, p2, p3, threshold=1e-6):
    n = np.cross(p2 - p1, p3 - p1)
    return np.linalg.norm(n) < threshold


def obtain_trans_matrix(p1, p2, p3):
    # Change of reference frame
    v1 = p2 - p1
    v2 = p3 - p2
    n = np.cross(v1, v2)   # plane normal vector

    # New axis
    z_prime = n / np.linalg.norm(n)
    x_prime = v1 / np.linalg.norm(v1)
    y_prime = np.cross(z_prime, x_prime)

    # Homogeneous transformation matrix
    R = np.array([x_prime, y_prime, z_prime])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -np.dot(R, p1)

    return T

def change_basis(vectors, T):
    n = len(vectors)
    new_column = np.ones([n, 1])
    vectors_h = np.hstack((vectors, new_column))
    vectors_prime = np.dot(T, vectors_h.T).T
    return vectors_prime[:, :3]

def return_basis(vectors_prime, T):
    T_inv = np.linalg.inv(T)
    n = len(vectors_prime)
    new_column = np.ones([n, 1])
    vectors_prime_h = np.hstack((vectors_prime, new_column))
    vectors = np.dot(T_inv, vectors_prime_h.T).T
    return vectors[:, :3]

def obtain_arc_2D(p1, p2, p3):
    T = obtain_trans_matrix(p1, p2, p3)

    given_points = np.array([p1, p2, p3])
    p1_prime, p2_prime, p3_prime = change_basis(given_points, T)   # Points in the plane

    # Middle points
    p4_prime = (p1_prime + p2_prime) / 2
    p5_prime = (p2_prime + p3_prime) / 2

    # Vectors of lines through middle points and perpendiculars to v1 and v2 in plane
    v1_prime = p2_prime - p1_prime
    v2_prime = p3_prime - p2_prime
    v4 = np.array([v1_prime[1], - v1_prime[0], 0])
    v5 = np.array([v2_prime[1], -v2_prime[0], 0])

    # Solving for the intersection
    A = np.array([v4[:2], -v5[:2]]).T
    b = p5_prime[:2] - p4_prime[:2]
    lam = np.linalg.solve(A, b)[1]
    center = p5_prime + lam * v5

    vr1 = p1_prime - center
    t1 = math.atan2(vr1[1], vr1[0])  # Angle from center to p1
    if t1 < 0:
        t1 += 2 * np.pi

    vr3 = p3_prime - center
    t3 = math.atan2(vr3[1], vr3[0]) # Angle from center to p2
    if t3 < 0:
        t3 += 2 * np.pi

    radius = np.linalg.norm(vr1)
    step = 0.005 / radius            # theta ~ S/R  and we want S = 0.005 (5 mm separation between points)

    def interpolate_angles(t1, t3, step = step):
        if t1 > t3:
            a = np.arange(t1, 2 * np.pi, step)
            b = np.arange(0, t3, step)
            ang = np.concatenate((a, b))
        else:
            ang = np.arange(t1, t3, step)

        return ang

    ang = interpolate_angles(t1, t3)

    c = np.cos(ang)
    s = np.sin(ang)
    zero = np.zeros(len(c))

    points = np.array([c, s, zero]).T * radius + center

    return points, center

def obtain_arc_3D(p1, p2, p3):
    T = obtain_trans_matrix(p1, p2, p3)
    points_2D, center_2D = obtain_arc_2D(p1, p2, p3)
    points_3D = return_basis(points_2D, T)
    #center_3D = return_basis(np.array([center_2D]), T)[0]

    return points_3D
