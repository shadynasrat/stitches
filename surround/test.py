import cv2
import numpy as np
import json

# create a black image
height = 1280*3
width = 720*3

with open('data.json', 'r') as f:
    data = json.load(f)

Matrix = np.array(data)

img = np.zeros((width, height), np.uint8)

# define the vertices of the trapezium masks
trapezium1 = np.array(Matrix[0], np.int32)
trapezium2 = np.array(Matrix[1], np.int32)
trapezium3 = np.array(Matrix[2], np.int32)
trapezium4 = np.array(Matrix[3], np.int32)
square     = np.array(Matrix[4], np.int32)

# fill the trapezium masks and square mask with white color
cv2.fillPoly(img, [trapezium1], (255, 255, 255))
cv2.fillPoly(img, [trapezium2], (0, 0, 0))
cv2.fillPoly(img, [trapezium3], (255, 255, 255))
cv2.fillPoly(img, [trapezium4], (0, 0, 0))
cv2.fillPoly(img, [square],     (255, 255, 255))

#img = cv2.resize(img, (1280, 800))

# display the masks
cv2.imshow('Masks', img)
cv2.waitKey(0)  # wait for a key press
cv2.destroyAllWindows()  # close all windows
