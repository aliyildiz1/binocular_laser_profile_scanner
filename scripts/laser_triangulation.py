#!/usr/bin/python3

import math
import numpy as np

# ----------------------------------------------------------
# CONSTANTS
# ----------------------------------------------------------
# Camera Specs
p_w = 600        # horizontal pixel number
p_h = 600        # vertical pixel number
p_size = 0.02    # pixel size
w = p_w * p_size # frame width
h = p_h * p_size # frame height
fc = 8           # center focal lenght

# scanner specs
l_c = 80                              # laser-camera distance
l_o = 70                              # laser-origin distance
beta = math.atan(l_c/l_o)             # Laser - Origin - Camera angle
f_o = math.sqrt(l_c**2 + l_o**2) - fc # F-origin distance
p_o_f = f_o*math.sin(beta)            # projection of F-origin distance

# ----------------------------------------------------------
# FUNCTIONS
# ----------------------------------------------------------
def calculate_multiple_pos(pos_array):
    # Position array must have 2 dimensions. If not, remove dimension.
    pos_array = np.squeeze(pos_array)

    calculated_pos = np.zeros((len(pos_array), 3), dtype="float64")

    for i in range(len(pos_array)):
        calculated_pos[i] = calculate_pos(pos_array[i])

    return calculated_pos
        
#First 2 elements of pos variable must be [x, y] coordinates      
def calculate_pos(pos): 
    x = pos[0]
    y = pos[1]

    cal_py = 1.33
    cal_pz = 1.0

    # Centering pixel coordinates
    c_x = (x - p_w / 2) * p_size + p_size / 2                              # camera x axis distance  
    c_y = (y - p_h / 2) * p_size + p_size / 2                              # camera y axis distance

    v_theta = math.atan(c_y/fc)                                            # vertical theta

    # Calculating positions according to origin coordinate system
    pz = f_o * math.sin(v_theta)/math.sin(beta + v_theta)                  # z axis coordinate
    py = (c_x*p_o_f*math.sin(beta))/(fc*math.sin(beta)+c_y*math.cos(beta)) # y axis coordinate

    # Translation of y-z coordinates from origin to laser coordinate system
    # Changing units from mm to m
    py = (-1 * py * cal_py) / 1000
    pz = ((-1 * pz * cal_pz) + l_o) / 1000

    # Since the measurement is made in the y-z plane(line laser plane), the x-axis is always zero
    return np.array([0, py, pz])