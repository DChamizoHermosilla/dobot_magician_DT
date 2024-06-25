#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import argparse
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

from functions import linear, extended_mask, calculate_iou, group_lines, intersec, px2coord

def get_color_mask(img, color):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if color == 'Red':
        lower_color = np.array([0,100,100])
        upper_color = np.array([10,255,255])
        lower_color2 = np.array([165, 100, 100])
        upper_color2 = np.array([180, 255, 255])
    '''
    if color == 'Orange':
        lower_color = np.array([10,100,100])
        upper_color = np.array([22,255,255])
    '''
    if color == 'Yellow':
        lower_color = np.array([22,100,100])
        upper_color = np.array([35,255,255])
    if color == 'Green':
        lower_color = np.array([35,100,100])
        upper_color = np.array([80,255,255])
    if color == 'Blue':
        lower_color = np.array([80, 100, 100])  # Umbral inferior para el color azul en el espacio HSV
        upper_color = np.array([135, 255, 255])
    '''
    if color == 'Purple':
        lower_color = np.array([135,100,100])
        upper_color = np.array([160,255,255])
    '''

    mask_color = cv2.inRange(hsv_img,lower_color,upper_color)

    if color == 'red':
        mask2 = cv2.inRange(hsv_img, lower_color2, upper_color2)
        mask_color = cv2.bitwise_or(mask_color, mask2) # Combinar ambas máscaras si es rojo

    return mask_color



def get_lines(img, h, w):

    borders = cv2.Canny(img, min_t, 255)                        # borders of cub
    borders_color = cv2.cvtColor(borders,cv2.COLOR_GRAY2BGR)       # Border in BGR format
    ba = borders.copy()  # Copy of the borders

    p=min(h, w)           # number of intersection lines for Hough Lines

    lp = []                # line points
    line_m = []              # slope of lines
    line_n = []              # intercept of lines


    while np.any(ba == 255):
        lines = cv2.HoughLines(ba, 1, np.pi/180, p, None, 0, 0)      # Lines of the image
        p-=1      # Decrease the number of intersection points

        if lines is not None:
            points=[]        # points of the line
            slope=[]         # slopes of the lines
            intercept=[]     # intercept of the lines
            line_matrix=[]   # line matrices

            # Get the equation of each line
            for i in range(len(lines)):
                rho = lines[i][0][0]       # minimum distance to line
                theta = lines[i][0][1]     # angle of x axis with rho
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho   # x coordinate (column)
                y0 = b * rho   # y coordinate (row)

                # Extended line
                x1 = int(x0 + 10000 * (-b))
                y1 = int(y0 + 10000 * (a))
                x2 = int(x0 - 10000 * (-b))
                y2 = int(y0 - 10000 * (a))

                pt1 = (x1, y1)                # point 1 of line
                pt2 = (x2, y2)                # point 2 of lines
                points.append([pt1, pt2])   # Add the points

                # Equation of the line and representation
                m, n, lm = linear(pt1, pt2, h, w)

                slope.append(m)           # Add the slope
                intercept.append(n)       # Add the intercept
                line_matrix.append(lm)    # Add the line matrix


            # Choosing the line that better represents the border
            mask_ba = ba == 255                                           # Points of the border
            mask_lines = [i == 255 for i in line_matrix]                  # Points of the line
            mask_px = [np.logical_and(mask_ba, i) for i in mask_lines]    # Points belonging to both

            min_iou = 0.2                          # min value of IoU (Intersection over Union)
            mg, I= group_lines(mask_px, min_iou)    # Group the lines according to IoU

            for i in range(len(mg)):
                npx = [np.count_nonzero(mask_px[k]) for k in mg[i]]   # number of pixels
                ind = np.argmax(npx)                                  # index of the highest npx of the group
                ind = mg[i][ind]                                      # index of the line with highest npx of the group

                # We increase the width of the line to check that the border is completely deleted
                nc = 1                                        # number of increased columns
                nr = 2                                        # number of increased rows
                exm = extended_mask(mask_lines[ind], nc, nr)    # Increased width

                # Create and apply the mask
                mask = mask_px[ind]    # Line that better represents the line
                mask[exm] = True       # Increasing the width of the line
                ba[mask] = 0             # Erase the border the line represents

                # Drawing line
                pt1 = points[ind][0]     # point 1 of the line
                pt2 = points[ind][1]     # point 2 of the line

                cv2.line(borders_color, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)

                # Saving equation of lines
                line_m.append(slope[ind])
                line_n.append(intercept[ind])

                '''
                cv2.line(bordes_color, pt1, pt2, (0,0,255), 1, cv2.LINE_AA)
                cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", borders_color)
                cv2.imwrite("output.png", borders_color)
                cv2.waitKey(0)
                cv2.imshow('a',ba)
                cv2.waitKey(0)
                '''

        if p==20:
            break

    '''
    cv2.imshow("Detected Lines (in red) - Standard Hough Line Transform", borders_color)
    cv2.imwrite("output.png", borders_color)
    cv2.waitKey(0)
    '''
    return line_m,line_n



def get_vertices(img, line_m, line_n, h, w, hc, L, F):
    # hc is the height of the camera
    # L is the side of the cube
    # F is the focal length

    # LINE INTERSECION POINTS
    n=len(line_m)

    int_points=[]
    for i in range(n):
        for j in range(i+1, n):
            points = intersec(line_m[i], line_m[j], line_n[i], line_n[j])
            if points[0] != None and points[1] != None:
                int_points.append(points)

    # INTERSECION POINTS INSIDE THE REGION OF INTEREST
    # Mask of the image (Pixels belonging to image)
    mask = img == 255

    # Increase the mask in case the vertices don't fall within the mask
    nc=5                   # number of increased columns
    nr=5                    # number of increased rows
    exm = extended_mask(mask,nc,nr)
    mask[exm] = 255           # Extended mask

    # Intersection points inside our mask
    points_in = []
    for p in int_points:
        if (p[1] >= 0 and p[1] <= h) and (p[0] >= 0 and p[0] <=w):
            if mask[p[1], p[0]]:
                points_in.append(p)

    # VERTICES
    # If 2 points are really close, they are the same vertex
    v_px = []
    while points_in:
        pi = points_in[0]
        points_in.remove(pi)
        x = [pi[0]]
        y = [pi[1]]
        points_aux = points_in.copy()
        for pj in points_aux:
            dx=abs(pi[0] - pj[0])
            dy=abs(pi[1] - pj[1])
            if dy <= 5 and dx <= 5:
                x.append(pj[0])
                y.append(pj[1])
                points_in.remove((pj))
        vx = int(np.mean(x))
        vy = int(np.mean(y))
        v_px.append((vx,vy))

    # Ordering the vertices according to their y (2nd component) and left to right (1st component)
    v_px = sorted(v_px, key = lambda x: (x[1], x[0]))
    v_px = np.array(v_px)

    # To calculate the 3D position, we need the pixels with respect to the center of the camera
    v_px2 = np.zeros_like(v_px)
    v_px2[:, 0] = - (v_px[:, 0] - w/2)
    v_px2[:, 1] = h/2 - v_px[:, 1]
    v_3D = px2coord(v_px2, hc, L, F)


    return v_px, v_3D




def get_center_orientation(v_3D, L):
    # According to how they were ordered, the diagonal is formed by the 1st and 4th points
    v1 = v_3D[0]
    v2 = v_3D[3]
    v1v2 = v2-v1    # Vector between the 2 vertex

    cf = v1 + v1v2 / 2      # face center
    C = cf + [0, 0, L/2]  # cube center

    v1v2_oriented = np.array([L, L, 0])
    alpha = (np.dot(v1v2,v1v2_oriented))/(np.linalg.norm(v1v2)*np.linalg.norm(v1v2_oriented))   # In radians

    return C, alpha



class robot_camera:
    def __init__(self, image_topic_name, color, camera_info_topic, hc, L):
        self.image_topic_name = image_topic_name
        self.color = color
        self.camera_info_topic = camera_info_topic
        self.height_cam = hc
        self.cube_side = L

        rospy.init_node("cube_detection")
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        rospy.Subscriber(self.image_topic_name, Image, self.camera_callback)
        self.cube_center = rospy.Publisher('/cube_center', Float32MultiArray, queue_size=1)
        self.cube_orientation = rospy.Publisher('/cube_orientation', Float32, queue_size=1)
        self.bridge = CvBridge()


    def camera_info_callback(self, msg):
        K = msg.K
        P = msg.P
        self.focal = K[0]



    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.h,self.w = img.shape[:2]

        mask_color = get_color_mask(img,self.color)

        if np.all(mask_color != 255):
            self.C = None
            self.alpha = None


        else:
            masked_img = cv2.bitwise_and(img, img, mask=mask_color)

            min_t = 100
            gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, min_t, 255, cv2.THRESH_BINARY)


            line_m, line_n = get_lines(thresh, self.h, self.w)
            v_px, v_3D = get_vertices(thresh, line_m, line_n, self.h, self.w,
                                      self.height_cam, self.cube_sidee, self.F)


            while len(v_px) < 4:
                min_t -= 5
                gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, min_t, 255, cv2.THRESH_BINARY)

                line_m, line_n = get_lines(thresh, self.h, self.w)
                v_px, v_3D = get_vertices(thresh, line_m, line_n, self.h, self.w,
                                          self.height_cam, self.cube_side, self.F)

            while len(v_px) > 4:
                min_t += 5
                gray = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, min_t, 255, cv2.THRESH_BINARY)

                line_m, line_n = get_lines(thresh, self.h, self.w)
                v_px, v_3D = get_vertices(thresh, line_m, line_n, self.h, self.w,
                                          self.height_cam, self.cube_side, self.F)


            self.v_px = v_px
            self.v_3D = v_3D
            self.C, self.alpha = get_center_orientation(self.v_3D, self.cube_side)

            cube_center_msg = Float32MultiArray(data = self.C)
            #cube_center_msg.data = [self.C[0], self.C[1], self.C[2]]
            #cube_center_msg.layout.dim = []
            self.cube_center.publish(cube_center_msg)

            cube_orientation_msg = Float32(data = self.alpha)
            #cube_orientation_msg.data = [self.alpha]
            #cube_orientation_msg.layout.dim = []
            self.cube_orientation.publish(cube_orientation_msg)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Obtaining position of cube from camera')
    parser.add_argument('--color', type=str, nargs='str', help='Cube color', required = True)
    parser.add_argument('--height_cam', type=float, help='Camera_height', required = True)
    parser.add_argument('--cube_side', type=float, help='Cube_side', required = True)

    args = parser.parse_args()

    cube = robot_camera('/dobot/dobot/camera/image_raw', args.color,
                        '/dobot/dobot/camera/camera_info', args.height_cam,
                        args.cube_side)

    rospy.sleep(1)  # Esperar un poco para que el callback tenga la oportunidad de ejecutarse
    rospy.spin()
