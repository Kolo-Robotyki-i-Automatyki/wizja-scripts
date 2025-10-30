# Doesn't work. Z16 is not natively supported.

import cv2
import numpy as np

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Z','1','6',' '))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
if ret:
    # Normalize for viewing (depth values are 16-bit)
    depth_vis = cv2.normalize(
        frame,
        None,
        0.0,
        255.0,
        cv2.NORM_MINMAX,
        dtype=cv2.CV_8U)
    cv2.imshow('Depth', depth_vis)
    cv2.waitKey(0)
cap.release()