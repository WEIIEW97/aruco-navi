import pyaruconavi
import cv2
import numpy as np


left_img = cv2.imread("aruco_capture/left_departure.png")
intrinsic = np.array([610.117, 608.71, 316.156, 249.345])
distortion = np.array([0, 0, 0, 0, 0])

calculator = pyaruconavi.PyCalc(intrinsic, distortion, 0.176)
ypr, trans = calculator.process(left_img)
print(ypr)
print(trans)
