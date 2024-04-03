import cv2
import numpy as np
from D405_lib import marker

marker_obj = marker()

while True:
    marker_obj.get_frames()
    marker_obj.simple_marker_detection()
    mask_img = marker_obj.mask_test(1)
    check_point = marker_obj.ray_casting_check(100 , 500, 1)
    
    # debuging  
    if check_point == True:
        cv2.circle(mask_img , (100,500) , 20 , (255, 255, 255) , 5)
    else :
        cv2.circle(mask_img , (100,500) , 20 , (0 , 0 , 0) , 5)


    merge_image = np.hstack((marker_obj.color_image , marker_obj.depth_colormap))

    print(check_point)

    cv2.namedWindow('masked image' , cv2.WINDOW_AUTOSIZE)
    cv2.imshow('masked image' , mask_img)
    key = cv2.waitKey(1)

    if key & 0XFF == ord('q') or key == 27:
        cv2.destroyAllWindows()

        break