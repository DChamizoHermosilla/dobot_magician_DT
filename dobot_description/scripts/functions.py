#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np


# Equation of linear function and representation in matrix
def linear(p1, p2, h, w):
    # p1 = point 1
    # p2 = point 2
    # h = height of image
    # w = width of image

    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    lm = np.zeros([h, w], int)

    if x1 == x2:
        # x=n
        m = None
        n = x1
        lm[:, x1] = 255
    else:
        # y=mx+n
        m = (y2 - y1) / (x2 - x1)
        n = y1 - m * x1
        # Line in pixels
        xp = np.arange(w)
        yp = m * xp + n
        yp = yp.astype(int)
        # Only the pixels that can appear in our image
        mask = (yp >= 0) & (yp <= h-1)
        xp = xp[mask]
        yp = yp[mask]
        lm[yp, xp] = 255

    return m,n,lm



# Increasing a mask in nc columns and nr rows
def extended_mask(m, nc, nr):
    # m = mask
    # nc = nº of columns
    # nr = nº of rows

    exm = np.zeros_like(m, dtype=bool)
    # Move left
    for k in range(1, nc + 1):
        shifted_left = np.roll(m, k, axis=1)
        shifted_left[:, :k] = False              # Delete elements in right column
        exm |= shifted_left
    # Move right
    for k in range(1, nc + 1):
        shifted_right = np.roll(m, -k, axis=1)
        shifted_right[:, -k:] = False            # Delete elements in left column
        exm |= shifted_right
    # Move downwards
    for k in range(1, nr + 1):
        shifted_down = np.roll(m, k, axis=0)
        shifted_down[:k, :] = False              # Delete elements in highest row
        exm |= shifted_down
    # Move upwards
    for k in range(1, nr + 1):
        shifted_up = np.roll(m, -k, axis=0)
        shifted_up[-k:, :] = False               # Delete elements in lowest row
        exm |= shifted_up
    return exm



# Calculation of IoU (Intersection over Union)
def calculate_iou(A, B):
    # A = matrix A
    # B = matrix B

    inter = np.logical_and(A, B)
    union = np.logical_or(A, B)
    sum_union = np.sum(union)

    if sum_union == 0:
        return 0  # Otra opción podría ser return np.nan
    else:
        iou = np.sum(inter) / sum_union
        return iou



# Grouping matrices according to IoU
def group_lines(m, min_iou):
    # m = list of matrices
    # min_iou = minimum IoU to group matrices

    n = len(m)              # nº of matrices
    ind = list(range(n))  # Indeces of the matrices that haven't been grouped
    mg = []               # Grouped matrices
    I = []                # List of IoUs

    while ind:               # While there are some non-grouped matrices
        sublist = []
        iou_ind = []
        i = ind[0]              # First non-grouped matrix
        sublist.append(i)       # Create its sublist
        ind.remove(i)           # Remove its index
        # Comparing the matrix with the non-grouped ones
        ind_aux = np.copy(ind)
        for j in ind_aux:
            iou = calculate_iou(m[i], m[j])   # IoU
            a = [i, j, iou]
            iou_ind.append(a)                 # Add the IoU
            if iou > min_iou:                 # If the IoU fullfils the condition
                sublist.append(j)             # j matrix is included to the sublist
                ind.remove(j)                 # Remove index of the j matrix

        for k in sublist[1:]:                 # Take other matrix from sublist
            ind_aux = np.copy(ind)
            for l in ind_aux:                 # Non-grouped matrix
                iou = calculate_iou(m[k], m[l])  # IoU
                a = [k, l, iou]
                iou_ind.append(a)             # Add the IoU
                if iou > min_iou:             # If the IoU fullfils the condition
                    sublist.append(l)         # l matrix is included to the sublist
                    ind.remove(l)             # Remove index of l matrix

        # Add the group of matrices to mg
        mg.append(sublist)
        I.append(iou_ind)

    return mg, I




# Intersecion points between 2 lines
def intersec(m1, m2, n1, n2):
    # m1 = line 1 slope (None if vertical)
    # m2 = line 2 slope (None if vertical)
    # n1 = line 1 intercept (Value of x if vertical)
    # n2 = line 2 intercept (Value of x if vertical)

    if (m1 == None and m2 == None) or (m1 == m2):   # Both lines are parallel
        return None, None

    else:
        if m1 == None and m2 != None:        # Line 1 is vertical
            x = n1
            y = m2 * x + n2
        elif m1 != None and m2 == None:      # Line 2 is vertical
            x = n2
            y = m1 * x + n1
        else:                      # Intersection of 2 affine lines
            x = (n2 - n1) / (m1 - m2)
            y = m1 * x + n1
        return int(x), int(y)



# Transform pixels to 3D coordinates
def px2coord(v_px, hc, L, F):
    # v_px = point in pixels
    # hc = height of camera (relative to floor)
    # L = height of point in 3D relative to floor (side of the cube)
    # F = camera focal distance

    v_px = np.array(v_px)
    F_column =  np.full((v_px.shape[0], 1), F)
    v_px = np.hstack((v_px, F_column))
    a = (hc - L) / F
    v_3D = a * v_px

    return v_3D



# Transform 3D coordinates into pixels
def coord2px(v_coord,F):
    # v_coord = point in 3D coordinates
    # F = camera focal distance

    v_coord= np.array(v_coord)
    a = F / v_coord[:,2]
    a = np.tile(a, (3, 1)).T
    v_px = a * v_coord
    v_px = v_px[:, :2]

    return v_px

