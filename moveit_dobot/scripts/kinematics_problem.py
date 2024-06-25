#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi

l0 = 0.051
l1 = 0.086999874
l2 = 0.134999185
l3 = 0.14700014
l4 = 0.059772341
l5 = 0.091478944

d0, a0 , alpha0 = l0, 0, 0
d1, a1, alpha1 = l1, 0, -pi/2
d2, a2, alpha2 = 0, l2, 0
d3, a3, alpha3 = 0, 0, 0
d4, a4, alpha4 = 0, l3, 0
d5, a5, alpha5 = 0, l4, pi/2
d6, a6, alpha6 = -l5, 0, pi

q1_min, q1_max = -pi/2, pi/2
q2_min, q2_max = 0, 85 * pi / 180
q3_min, q3_max = 0, 100 * pi / 180
q4_min, q4_max = -pi/2, pi/2
zmin = 0.0125


def A_revolute(t, d, a, alpha):
    ct = np.cos(t)
    st = np.sin(t)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    A = np.array([[ct, -st*ca, st*sa, a*ct],
                  [st, ct*ca, -ct*sa, a*st],
                  [0, sa, ca, d],
                  [0, 0, 0, 1]])
    return A


def T_dobot(t1, t2, t4, t6):
    A0 = A_revolute(0, d0, a0, alpha0)
    A1 = A_revolute(t1, d1, a1, alpha1)
    A2 = A_revolute(t2 - pi/2, d2, a2, alpha2)
    A3 = A_revolute(-t2, d3, a3, alpha3)
    A4 = A_revolute (t4 + 80 * pi /180, d4, a4, alpha4)
    A5 = A_revolute(-t4 + 10 * pi /180, d5, a5, alpha5)
    A6 = A_revolute(t6, d6, a6, alpha6)

    T = A0 @ A1 @ A2 @ A3 @ A4 @ A5 @ A6

    return T


def obtain_q1(x):
    q1 = np.arctan2(x[1], x[0])
    
    if abs(q1) > q1_max:
        q1 = None
    return q1


def change_coord(x):
    r = np.sqrt(x[0]**2 + x[1]**2)
    r -= l4
    z = x[2] - l0 - l1 + l5
    x2 = [r, z]
    return x2  


def functions_q2(q3, x):
    y1 = np.zeros_like(q3)   # q2
    y2 = np.zeros_like(q3)

    arg1 = (x[0] - l3 * np.sin(q3 + 4/9 * pi)) / l2
    arg2 = (x[1] - l3 * np.cos(q3 + 4/9 * pi)) / l2
    ind1 = (arg1 <= 1) & (arg1 >= -1)
    ind2 = (arg2 <= 1) & (arg2 >= -1)

    y1[ind1] = np.arcsin(arg1[ind1])
    y1[~ind1] = np.nan
    y2[ind2] = np.arccos(arg2[ind2])
    y2[~ind2] = np.nan
    
    return y1, y2


def exit_solution(q3, x):
    y1, y2 = functions_q2(q3, x)   # q2
    
    ind1 = np.where(~np.isnan(y1))[0]         # points where y1 != nan
    ind2 = np.where(~np.isnan(y2))[0]         # points where y2 != nan
    common_ind = np.intersect1d(ind1, ind2)   # points where  y1 and y2 != nan
    
    valid_y1 = y1[common_ind]        # valid values of y1
    valid_y2 = y2[common_ind]        # valid values of y2
    valid_q3 = q3[common_ind]        # valid values of q3
    
    y3 = valid_y1 - valid_y2                # Difference between y1 and y2
    diff = np.diff(np.sign(y3))             # Transform y3 into -1 (negative), +1 (positive) or 0 and get the difference between consecutive elements
    sol_ind = np.where(abs(diff) != 0)      # If the difference is != 0, then there is a change in sign (solution)
    
    if len(sol_ind[0]) == 0:
        q2_sol = None
        q3_sol = None
        
    else:
        q2_sol = valid_y1[sol_ind]    # Solutions (approximation)
        q3_sol = valid_q3[sol_ind]
            
    return q2_sol, q3_sol


def obtain_q2q3(x):
    from scipy import optimize
    r = x[0]
    z = x[1]
    
    # Exist solution)
    q3 = np.linspace(q3_min, q3_max, 10**5)
    q2_sol0, q3_sol0 = exit_solution(q3, x)    
    
    if q2_sol0 is None:
        sol = np.array([None, None])    # No solutions
        
    else:
        x0 = np.column_stack((q2_sol0, q3_sol0))      # Initial values for Newton-Raphson
        x0 = x0.reshape(2)
            
        def f(x):                                     # Defining functions for Newton-Raphson
            c2 = np.cos(x[0])
            s2 = np.sin(x[0])
            c3 = np.cos(x[1] + 4/9 * pi)
            s3 = np.sin(x[1] + 4/9 * pi)
                
            f1 = l2 * s2 + l3 * s3 - r
            f2 = l2 * c2 + l3 * c3 - z
            return [f1, f2]
        
        
        sol = optimize.fsolve(f, x0)       # Newton-Raphson solutions
        
        # Check values are within limits
        sol = sol[(sol[0] >= q2_min) & (sol[0] <= q2_max) & (sol[1] >= q3_min) & (sol[1] <= q3_max)]
        if sol.size == 0:
            sol = np.array([None, None])
        
        sol = sol.reshape(2)

    return sol


def obtain_q4(b, q1):
    q4 = b - q1
    if abs(q4) > q4_max:
        q4 = None
    return q4



def angle2position(q1, q2, q3, q4):
    x0 = T_dobot(q1, q2, q3, q4) @ np.array([0, 0, 0, 1])
    b = q1 + q4
    return x0[0:3], b



def position2angle(x, b):
    q1 = obtain_q1(x)
    if q1 is None:
        sol = np.array([None] * 4)
    else:
        q4 = obtain_q4(b, q1)
        if q4 is None:
            sol = np.array([None] * 4)
        else:
            x2 = change_coord(x)
            q2q3 = obtain_q2q3(x2)
            if q2q3[0] is None:
                sol = np.array([None] * 4)
            else:
                q1_col = np.array([q1])
                q4_col = np.array([q4])
                sol = np.hstack(([q1], q2q3, [q4]))

    return sol
