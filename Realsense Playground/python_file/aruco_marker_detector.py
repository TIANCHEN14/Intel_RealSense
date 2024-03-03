#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

# aruco marker dictionary and intilized
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
aruco_detector = cv2.aruco.ArucoDetector(dictionary)

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print (device.sensors)


# config the sensor information
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print('Depth Scale is: ' , depth_scale)

# we will remove all the background objects more than 1 meters
clipping_distance_in_meters = 1
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The 'align_to' is the stream type to which we plan to align depth frames
align_to = rs.stream.color
align = rs.align(align_to)


# streaming loop
try:
    while True:
        # get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640X360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        ##########################################################################################################################
        # aruco marker detector and depth display
        marker_corner, marker_id, rej = aruco_detector.detectMarkers(color_image)
        cv2.aruco.drawDetectedMarkers(color_image , marker_corner , marker_id)

        # mask the color map with the detect corners
        if marker_id is not None:
            for index , m_id in enumerate(marker_id):
                marker_corner_np = np.array(marker_corner[index] , dtype = np.int32)

                print(m_id)
                print(marker_corner_np)
                print('shape of marker corner' , np.shape(marker_corner_np))

                # creating an mask for the marker corners
                mask = np.zeros(depth_image.shape , dtype= np.uint8)
                cv2.fillPoly(mask , marker_corner_np , 255)
                roi_depth = cv2.bitwise_and(depth_image, depth_image , mask = mask)

                # calculate the average depth for ROI
                avg_depth = np.sum(roi_depth) / np.count_nonzero(roi_depth)

                print('avg_depth is ' , avg_depth)

        ##########################################################################################################################

        # render images:
        # depth align to color on left
        # depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image , alpha = 0.04) , cv2.COLORMAP_JET)
        cv2.aruco.drawDetectedMarkers(depth_colormap , marker_corner , marker_id)

        org = (200, 200)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        color = (255, 0, 0)
        thickness = 2
        color_image = cv2.putText(color_image , str(avg_depth), org, font, fontScale , color , thickness , cv2.LINE_AA)


        images = np.hstack((color_image , depth_colormap ))

        cv2.namedWindow("Align Example" , cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example' , images)
        key = cv2.waitKey(1)

        # press esc or 'q' to close the image window
        if key & 0XFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()










