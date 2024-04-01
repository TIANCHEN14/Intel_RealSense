import cv2
import numpy as np
from D405_lib import marker

marker_obj = marker()

while True:
    marker_obj.get_frames()
    marker_obj.simple_marker_detection()
    mask_img = marker_obj.mask_test(2)

    merge_image = np.hstack((marker_obj.color_image , marker_obj.depth_colormap))

    cv2.namedWindow('merge image' , cv2.WINDOW_AUTOSIZE)
    cv2.imshow('merge image' , mask_img)
    key = cv2.waitKey(1)

    if key & 0XFF == ord('q') or key == 27:
        cv2.destroyAllWindows()

        break