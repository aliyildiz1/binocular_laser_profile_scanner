#!/usr/bin/python3

import numpy as np
import cv2

def thin(img):
    mask = cv2.inRange(img, np.array([0, 0, 220]), np.array([50, 50, 255]) )

    # non zero pixel coordinates
    pts = cv2.findNonZero(mask)

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
    
    return output_img