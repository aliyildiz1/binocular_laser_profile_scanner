#!/usr/bin/python3

import math

x = 300
y = 300

p_w = 600 # horizontal pixel number
p_h = 600 # vertical pixel number
p_size = 0.02 # pixel size
w = p_w * p_size # frame width
h = p_h * p_size # frame height

c_x = (x - p_w / 2) * p_size + p_size / 2 # camera x axis distance  
c_y = (y - p_h / 2) * p_size + p_size / 2 # camera y axis distance

fc = 8 # center focal lenght
l_c = 80 # laser-camera distance
l_o = 70 # laser-origin distance
beta = math.atan(l_c/l_o) # Laser - Origin - Camera angle
v_theta = math.atan(c_y/fc) # vertical theta
h_theta = math.atan(c_x/fc)
theta = math.atan(math.sqrt(c_x**2 + c_y**2)/fc) # theta
v_fov = 2 * math.atan(h / (2 * fc)) # vertical FOV

f_o = math.sqrt(l_c**2 + l_o**2) - fc

pz = f_o * math.sin(v_theta)/math.sin(beta + v_theta)

o_c = f_o*math.sin(beta)

py = (c_x*o_c*math.sin(beta))/(fc*math.sin(beta)+c_y*math.cos(beta))

print("x: ", x)
print("y: ", y)
print("Pz: ", -pz)
print("Py: ", -py)