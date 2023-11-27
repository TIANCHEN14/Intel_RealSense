###########################################
### simple aruco mark depth measurement ###
###########################################

# here is the part to import all the nessary library
import pyrealsense2 as rs
import numpy as np
import cv2

# here is part to define all the parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
param = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary , param)

# create an pipeline 
pipeline = rs.pipeline()

# create an configuation for the pipeline
config = rs.config()

# get device info from the pipeline
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# check RGB sensors to make sure they are avaliable
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == "RGB Camera":
        found_rgb = True
        break

if not found_rgb:
    print("This program need to have RGB with depth camera")
    exit(0)

# config the sensor information
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# start streaming
profile = pipeline.start(config)

# getting the scale for the depth sensor
# this object only contain static information nothing data stream related
# this is the optional part for it
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print('Depth Scale is: ' , depth_scale)

# create an align object
# rs.align allow us to perform alignment of the depth frame to RGB frame
align_to = rs.stream.color
align = rs.align(align_to)

try:
    while True:
        # get frameset of color and depth
        frames = pipeline.wait_for_frames()
        #depth_frame = frames.get_depth_frame()

        # align the depth frame to color frame
        aligned_frame = align.process(frames)

        # get aligned frames
        aligned_depth_frame = aligned_frame.get_depth_frame()
        color_frame = aligned_frame.get_color_frame()

        # validate that both frame are valid
        if not aligned_depth_frame or not color_frame:
            continue
        
        # this is the part to create all the images that is aligned
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # here is the part to generate all the markers
        marker_corners, marker_Ids, rej = detector.detectMarkers(gray_image)

        # create an marker info dict
        marker_info = {}

        if marker_Ids is not None:
            for i in range(len(marker_Ids)):
                marker_id = marker_Ids[i][0] # extract marker ID
                corner_point = marker_corners[i][0] # extract marker corners
                # we combine all the marker id and corner points in the marker_info dict
                marker_info[marker_id] = corner_point

            if 0 in marker_Ids:
                #print('Yes')
                cv2.aruco.drawDetectedMarkers(color_image, marker_corners, marker_Ids)
                #print(marker_corners)
        
        # put text on the video streams for easy lookup
        


        #image = np.hstack((color_image , grey_image))
        cv2.namedWindow('Color Image' , cv2.WINDOW_NORMAL)
        cv2.imshow('Color Image' , color_image)

        if cv2.waitKey(1) == ord('q'):
            break

finally:

    # stop stream
    pipeline.stop()




