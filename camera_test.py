import cv2

cap = cv2.VideoCapture(6)  # /dev/video6
ret, frame = cap.read()
if ret:
    cv2.imshow('Camera', frame)
    cv2.waitKey(0)
cap.release()