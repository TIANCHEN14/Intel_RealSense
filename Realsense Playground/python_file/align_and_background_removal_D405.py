#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2

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

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153

        # depth image is 1 channel and color is 3 channels 
        # Reason behind that is to be able to compare depth image and color image to be the same demation 
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image)) 
        #bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d < 0), grey_color , color_image)

        # render images:
        # depth align to color on left
        # depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image , alpha = 0.04) , cv2.COLORMAP_JET)
        #images = np.hstack((color_image , depth_colormap))

        blend_images = cv2.addWeighted(color_image , 0.5 , depth_colormap , 0.5 , 0.0)
        images = np.hstack((blend_images , depth_colormap))

        cv2.namedWindow("Align Example" , cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Align Example' , images)
        key = cv2.waitKey(1)

        # press esc or 'q' to close the image window
        if key & 0XFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()










