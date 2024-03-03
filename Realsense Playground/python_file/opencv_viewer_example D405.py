import pyrealsense2 as rs
import numpy as np
import cv2

# config pipeline for both depth and RGB channels
pipeline = rs.pipeline()
config = rs.config()

# get device product line for setting up support resolution
pipeline_warpper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_warpper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# here is the part to check to see if the device sensor have been found or not
#found_rgb = False
#for s in device.sensors:
#    if s.get_info(rs.camera_info.name) == 'RGB Camera':
#        found_rgb = True
#        break

#if not found_rgb:
#    print('The demo requires Depth camera with Color Sensor')
#    exit(0)

# here is the part to start the video stream
config.enable_stream(rs.stream.depth , 640 , 480 , rs.format.z16 , 30)
config.enable_stream(rs.stream.color , 640 , 480 , rs.format.bgr8 , 30)

pipeline.start(config)


try:
    while True:

        # wait for coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        
        # convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03) , cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # if depth and color resolution are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resize_color_image = cv2.resize(color_image, dsize = (depth_colormap_dim[1] , depth_colormap_dim[0]) , interpolation = cv2.INTER_AREA)
            images = np.hstack((resize_color_image, depth_colormap))

        else:
            images = np.hstack((color_image, depth_colormap))
        

        #images = np.array(depth_colormap)
        # show images
        cv2.namedWindow('Realsense' , cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Realsense' , images)
       # cv2.waitKey(1)
        

        if cv2.waitKey(1) == ord('q'):
            break

finally:

    # Stop stream
    pipeline.stop()