import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# check capture
if not cap.isOpened():
    print("can not open camera")
    exit()

while True:
    # capture frame and frame
    ret, frame = cap.read()

    cv2.imshow("intel Realsense" , frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


