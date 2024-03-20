#####################################################
###      Underwing Measuring Tool Library         ###
#####################################################

# This library will handle the camera connection and all the measurement math in the backgroud
# It will have an marker class that contain the phyical image and the information that is processed by this library

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib as plt

# Marker class to help with processing information of the image
class marker:
    
    # initialization of the image sensors and store all the information
    # in self object it contain following object
    # self{
    #       marker_dict         Type : python dict
    #       color_image         Type : np.array
    #       depth_image         Type : np.array
    #       pipeline            Type : rs object
    #       align               Type : rs object
    #}

    def __init__(self):
        
        # Aruco marker dictionary and intization
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_detector = cv2.aruco.ArucoDetector(dictionary)

        # initlizating intel realsense cameras
        # define pipeline as global object so other function can access that
        self.pipeline = rs.pipeline()

        # Create a config pipeline to stream
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        # config the sensor information
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale 
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()

        # create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        align_to = rs.stream.color
        
        # make align an global object so all the other class can access it 
        self.align = rs.align(align_to)

        # initlized marker dictorary 
        self.marker_dict = {}
        self.color_image = None
        self.depth_image = None
        self.depth_colormap = None
        self.text_image = None
        self.mask_image = None
    

    # generate color frame and depth frame
    def get_frames(self):

        # get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        # align depth frame to color frame
        aligned_frames = self.align.process(frames)
        
        # get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        # Validate the both frame are valid
        if not aligned_depth_frame or not aligned_color_frame:
            return False
        
        # here is the part to pass all the image into the self object  
        self.color_image = np.asanyarray(aligned_color_frame.get_data())
        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image , alpha = 0.04) , cv2.COLORMAP_JET)
        
        return True
    
    # simple marker distance detection
    # no filter pure raw images
    def simple_marker_detection(self):

        # generated aruco marker with aruco marker detector 
        marker_corners, marker_ids, rej = self.aruco_detector(self.color_image)
        
        # draw marker on images
        cv2.aruco.drawDetectedMarkers(self.color_image , marker_corners, marker_ids)
        cv2.aruco.drawDetectedMarkers(self.color_image , marker_corners, marker_ids)

        # error check for markers
        if marker_ids is not None:

            # loop though all the marker ids
            for index , id in enumerate(marker_ids):

                # load all markerc corner into an np array
                marker_corner_np = np.array(marker_corners[index] , dtype = np.int32)

                # create an mask for the marker corners
                mask = np.zeros(self.depth_image.shape , dtype = np.int8)
                cv2.fillPoly(mask, marker_corner_np, 255)
                roi_depth = cv2.bitwise_and(self.depth_image , self.depth_image , mask = mask)

                # calculate the average depth for ROI
                avg_depth = np.sum(roi_depth) / np.count_nonzero(roi_depth)

                # append information into the marker dictionary
                self.marker_dict[int(id)] = avg_depth


            

        



