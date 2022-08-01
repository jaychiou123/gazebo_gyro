import cv2 
import numpy as np


img=cv2.imread("map_1.png")

contours =np.load("walls.npy")
cv2.drawContours(img, contours, -1, (0,255,0), 3)
cv2.imwrite("debugging.png", img)
