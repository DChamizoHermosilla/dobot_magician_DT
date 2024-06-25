#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import argparse

parser = argparse.ArgumentParser(description='Mostrar una imagen.')
parser.add_argument('imagen', type=str, help='Nombre de la imagen a mostrar.')
args = parser.parse_args()
img = cv2.imread(args.imagen)

cv2.imshow('Imagen', img)
while not rospy.is_shutdown():
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # Presiona Esc para salir
        break

cv2.destroyAllWindows()
