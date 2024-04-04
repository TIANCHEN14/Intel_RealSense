#####################################################
###      Underwing Measuring Tool Library         ###
#####################################################

# This library will handle the camera connection and all the measurement math in the backgroud
# It will have an marker class that contain the phyical image and the information that is processed by this library

import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib as plt
import sys

# Marker class to help with processing information of the image
class marker:
    
    ###############################################
    ###             Constructors                ###
    ###############################################
        ################################################################################################################################################
    # initialization of the image sensors and store all the information
    # in self object it contain following object
    # self{
    #       marker_dict         Type : python dict
    #       color_image         Type : np.array
    #       depth_image         Type : np.array
    #       depth_colormap      Type : np.array
    #       text_image          Type : np.array
    #       mask_image          Type : np.array
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

    ##################################################
    ###             Public Function                ###
    ##################################################
        ################################################################################################################################################
    
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
        marker_corners, marker_ids, rej = self.aruco_detector.detectMarkers(self.color_image)
        
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
                temp_dict = {}
                temp_dict['depth'] = avg_depth
                temp_dict['corner_0'] = marker_corners[index][0][0]
                temp_dict['corner_1'] = marker_corners[index][0][1]
                temp_dict['corner_2'] = marker_corners[index][0][2]
                temp_dict['corner_3'] = marker_corners[index][0][3]
                self.marker_dict[int(id)] = temp_dict

        # clear the dictionary 
        # marker_dict.clear()
                

    # generate an empty frame and put the ID information into it
    def txt_image_generation(self):

        # generate an empty frame with the size of twice of width orginal image and the same height
        id_background = np.zeros((720 , 2560 , 3) , dtype = np.uint8)

        # defining text properity
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        color = (255, 255, 255)
        thickness = 2
        display_test = "new text \n"

        # loop though all the dict and put it in image
        if self.marker_dict is not None:
            for i, k in enumerate(self.marker_dict):
                if k < 20:
                    display_text = 'ID : ' + str(k) + ' D : ' + str(self.marker_dict[k]) + '\n'
                    org = (200 , 100*(i+1))
                    self.text_image = cv2.putText(id_background , display_text , org , font , fontScale , color , thickness , cv2.LINE_AA)


    # get an particular id's mask image 
    # just an testing script for testing all the holes in the marker
    def mask_test(self, id):

        # create an maske image
        mask = np.zeros(self.depth_image.shape , dtype = np.uint8)
        
        # check to see if marker is in the dictionary
        if id in self.marker_dict:
             
            # import all the object into an np array 
            marker_corner_0 = np.array(self.marker_dict[id]['corner_0'] , dtype = np.int32)
            marker_corner_0 = np.expand_dims(marker_corner_0 , axis = 0)
            marker_corner_1 = np.array(self.marker_dict[id]['corner_1'] , dtype = np.int32)
            marker_corner_1 = np.expand_dims(marker_corner_1 , axis = 0)
            marker_corner_2 = np.array(self.marker_dict[id]['corner_2'] , dtype = np.int32)
            marker_corner_2 = np.expand_dims(marker_corner_2 , axis = 0)
            marker_corner_3 = np.array(self.marker_dict[id]['corner_3'] , dtype = np.int32)
            marker_corner_3 = np.expand_dims(marker_corner_3 , axis = 0)

            # concatenate the numpy array as an single 2X4 array
            marker_corner_np = np.concatenate((marker_corner_0 , marker_corner_1 , marker_corner_2 , marker_corner_3) , axis = 0)

            # create an poly fill base on the marker corners
            cv2.fillPoly(mask , [marker_corner_np] , 255)

            # roi depth mask for image
            mask = cv2.bitwise_and(self.depth_image , self.depth_image , mask = mask)

            # apply color map for the mask image
            mask = cv2.applyColorMap(cv2.convertScaleAbs(mask , alpha = 0.04) , cv2.COLORMAP_JET)
        
        return mask
    

    # here is the function that will generate the mask image and also plot the histogram for the depth
    # Base on the historgram it will tell us how much of the image have holes in it
    def mask_hist_plot(self, id):
    # create an maske image
        mask = np.zeros(self.depth_image.shape , dtype = np.uint8)
        
        # check to see if marker is in the dictionary
        if id in self.marker_dict:
             
            # import all the object into an np array 
            marker_corner_0 = np.array(self.marker_dict[id]['corner_0'] , dtype = np.int32)
            marker_corner_0 = np.expand_dims(marker_corner_0 , axis = 0)
            marker_corner_1 = np.array(self.marker_dict[id]['corner_1'] , dtype = np.int32)
            marker_corner_1 = np.expand_dims(marker_corner_1 , axis = 0)
            marker_corner_2 = np.array(self.marker_dict[id]['corner_2'] , dtype = np.int32)
            marker_corner_2 = np.expand_dims(marker_corner_2 , axis = 0)
            marker_corner_3 = np.array(self.marker_dict[id]['corner_3'] , dtype = np.int32)
            marker_corner_3 = np.expand_dims(marker_corner_3 , axis = 0)

            # concatenate the numpy array as an single 2X4 array
            marker_corner_np = np.concatenate((marker_corner_0 , marker_corner_1 , marker_corner_2 , marker_corner_3) , axis = 0)

            # create an poly fill base on the marker corners
            cv2.fillPoly(mask , [marker_corner_np] , 255)
            
            # ray casting algrotium to find all the pixel information within the bounding box



            # roi depth mask for image
            mask = cv2.bitwise_and(self.depth_image , self.depth_image , mask = mask)

            # apply color map for the mask image
            mask = cv2.applyColorMap(cv2.convertScaleAbs(mask , alpha = 0.04) , cv2.COLORMAP_JET)
        
        return mask


    ###################################################
    ###             Private Function                ###
    ###################################################
    ################################################################################################################################################

    # ray casting function to check see if pixel is within the polygon
    # looking for an specific aruco marker in the marker dict
    def ray_casting_check(self, point_x , point_y ,id):

        # we start from outside of the polygon            
        # states variable declear
        inside = False
        
        # error check make sure the ID we passed in is detected
        if id in self.marker_dict:
            

            # import all the object into an np array 
            marker_corner_0 = np.array(self.marker_dict[id]['corner_0'] , dtype = np.int32)
            marker_corner_0 = np.expand_dims(marker_corner_0 , axis = 0)
            marker_corner_1 = np.array(self.marker_dict[id]['corner_1'] , dtype = np.int32)
            marker_corner_1 = np.expand_dims(marker_corner_1 , axis = 0)
            marker_corner_2 = np.array(self.marker_dict[id]['corner_2'] , dtype = np.int32)
            marker_corner_2 = np.expand_dims(marker_corner_2 , axis = 0)
            marker_corner_3 = np.array(self.marker_dict[id]['corner_3'] , dtype = np.int32)
            marker_corner_3 = np.expand_dims(marker_corner_3 , axis = 0)

            # concatenate the numpy array as an single 2X4 array
            marker_corner_np = np.concatenate((marker_corner_0 , marker_corner_1 , marker_corner_2 , marker_corner_3) , axis = 0)
            
            # pass corners into the sorting algorithms to get them in an rotation oriantation
            marker_corner_np = self.__corner_sorting(marker_corner_np)
            print(marker_corner_np)



            # loop though all the corners
            n = marker_corner_np.shape[0]

            for i in range(n):

                # import the first two point
                p1x = marker_corner_np[i-1][0]
                p1y = marker_corner_np[i-1][1]
                p2x = marker_corner_np[i][0]
                p2y = marker_corner_np[i][1]

                # compare the y value to make sure p1y is the lower point
                if p1y > p2y:
                    p1x , p1y , p2x , p2y = p2x , p2y , p1x , p1y 
                
                # check if the point is the same height as p1y and p2y
                #if point_y == p1y or point_y == p2y:
                    
                    # add an fraction on it to make sure when we calculate the intercept there will not be divided by 0
                    #point_y = point_y + 0.001

                # check intercept on the horiztal ray casting
                if point_y > p1y and point_y <= p2y:

                    # check if the points are within the bounding box range to save compute time
                    if point_x <= max(p1x , p2x):
                        
                        # make sure that we are not going to divide 0
                        if p1y != p2y and point_y != p1y:
                            
                            # check the slope to make up the odd even times that the ray passing by
                            m_edge = (p2x - p1x) / (p2y - p1y)
                            m_point = (point_x - p1x) / (point_y - p1y)

                        # check the slope to see we just need to make sure either one is bigger to get and odd number
                    
                        if m_point < m_edge:
                            inside = not inside
                        
                    
                        print("intercept with " , i-1 ,  i)

                    

            
        else:
            print("Marker not in class")


        return inside

    
    # sorting out corners orination
    # We will calculate the centroid and check the angle for each point relative to centroid 
    # Sort all the point base on angles
    def __corner_sorting(self, corners):
        
        # calculate the centroid of the polygon
        centriod = np.mean(corners, axis = 0)

        # calculate the angle to each point relative to centroid
        angles = np.arctan2(corners[:,1] - centriod[1] , corners[:,0] - centriod[0])

        # sort points by angles
        sort_index = np.argsort(angles)

        # reindex the points and return them
        return corners[sort_index]
        



            

        



