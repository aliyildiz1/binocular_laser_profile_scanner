#!/usr/bin/python3

import numpy as np
import cv2

from time import time

def find_center_pixels(left_cam_img, right_cam_img):
    # vertical flip
    left_cam_img = cv2.flip(left_cam_img, 0)
    
    left_mask = cv2.inRange(left_cam_img, np.array([0, 0, 220]), np.array([50, 50, 255]))
    right_mask = cv2.inRange(right_cam_img, np.array([0, 0, 220]), np.array([50, 50, 255]))
    mask = cv2.bitwise_or(left_mask, right_mask)

    _, width  = mask.shape
    
    mean_y = np.zeros((width*3, 2))

    sum = 0
    sum_count = 0
    centered_pixel_count = 0
    for y in range(width):
        column = mask[:, y]
        non_zero_pixels = np.where(column == 255)

        if np.size(non_zero_pixels) > 0:

            for i in range(np.size(non_zero_pixels)):

                if i != np.size(non_zero_pixels)-1:
                    diff = non_zero_pixels[0][i+1] - non_zero_pixels[0][i]
                else:
                    diff = 0
                
                sum += non_zero_pixels[0][i]
                sum_count += 1

                if diff != 1:
                    mean = sum/sum_count
                    mean_y[centered_pixel_count] = [y, mean]

                    sum = 0
                    sum_count = 0
                    centered_pixel_count += 1

    mean_pixels = np.zeros((centered_pixel_count,2))

    for y in range(centered_pixel_count):
        if mean_y[y][1] != 0:
            mean_pixels[y] = mean_y[y]
        else:
            break

    return mean_pixels