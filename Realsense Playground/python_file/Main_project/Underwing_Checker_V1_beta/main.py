###################################################
###     Beta Version for initial release        ###
###################################################

# this is the part to build an Web UI for this application
# all the math will be handle in the D405_library

import cv2
from D405_lib import marker
import numpy as np
import streamlit as st
import threading
from queue import Queue
import time

# First queue for the intel realsense frame
frame_queue = Queue(maxsize=1)

# Second queue for the live data
data_queue = Queue(maxsize=1)

# inititized marker object
marker_instant = marker()

# Here is the part to genreate all the frame
def depth_frame_generation():

    try:
        # thread function for the  capture the new frames
        rs_status = marker_instant.get_frames()

        # check the status of see if that frame exist
        if rs_status:

            # marker detector
            marker_instant.simple_marker_detection()

    except:
        # 
        print('yes')

        
