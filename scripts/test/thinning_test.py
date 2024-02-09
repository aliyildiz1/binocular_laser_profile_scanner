#!/usr/bin/python3

import numpy as np
import math
import cv2
"""
def nothing(x):
    pass

cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - B", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - G", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - R", "Trackbars", 220, 255, nothing)
cv2.createTrackbar("U - B", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - G", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - R", "Trackbars", 255, 255, nothing)

while True:
    img = cv2.imread("src/binocular_laser_profile_scanner/scripts/right_cam_image.png")

    # ----------------------------------------------------------
    # COLOR FILTERING
    # ----------------------------------------------------------

    cv2.namedWindow("Trackbars")
    l_b = cv2.getTrackbarPos("L - B", "Trackbars")
    l_g = cv2.getTrackbarPos("L - G", "Trackbars")
    l_r = cv2.getTrackbarPos("L - R", "Trackbars")
    u_b = cv2.getTrackbarPos("U - B", "Trackbars")
    u_g = cv2.getTrackbarPos("U - G", "Trackbars")
    u_r = cv2.getTrackbarPos("U - R", "Trackbars")

    lower_red = np.array([l_b, l_g, l_r])
    upper_red = np.array([u_b, u_g, u_r])

    mask = cv2.inRange(img, lower_red, upper_red)

    result = cv2.bitwise_and(img, img, mask=mask)

    # ----------------------------------------------------------
    # THINNING
    # ----------------------------------------------------------
    height, width, channels = img.shape

    # non zero pixel coordinates
    pts = cv2.findNonZero(mask)

    if cv2.findNonZero(img) is not None:
        # Connected component analysis
        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=4)



    cv2.imshow("img", img)
    # cv2.imshow("hsv", hsv)
    cv2.imshow("mask", mask)
    cv2.imshow("result", result)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

cv2.destroyAllWindows()
"""

img = cv2.imread("src/binocular_laser_profile_scanner/scripts/right_cam_image.png")
mask = cv2.inRange(img, np.array([0, 0, 220]), np.array([50, 50, 255]) )

# non zero pixel coordinates
pts = cv2.findNonZero(mask)
"""
İnceltme algoritması
1) Eğer görüntüde maskelenmiş bileşenler varsa işleme devam et
2) görüntüye bağlantılı bileşen analizi uygula
3) her bir objeyi incelt
"""
output_img = np.zeros(mask.shape, dtype="uint8")

if pts is not None:
    # Connected component analysis
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    height = img.shape[0]
    width = img.shape[1]
    
    for label in range(nb_components-1):
        p_index = []
        for w in range(width):
            p_index_vertical = []
            for h in range(height):
                if output[h][w] == label+1:
                    p_index_vertical.append(h)
            
            if len(p_index_vertical) != 0:
                p_mean = sum(p_index_vertical)//len(p_index_vertical)
                p_index.append((w, p_mean))

            n = len(p_index)
            if len(p_index) > 1:
                cv2.line(output_img, p_index[n-2], p_index[n-1], 255, 1)
            
    

cv2.imshow("output_img", output_img)
cv2.imshow("mask", mask)
cv2.waitKey(0)
cv2.destroyAllWindows()