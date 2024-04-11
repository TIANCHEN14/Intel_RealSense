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
from pages import live_feed, center_depth

# Initizlize queues for frame data and depth
frame_queue = Queue(maxsize=1)
depth_queue = Queue(maxsize=1)

# This function will be running on an different threads
# mainly to handle the intel realsense cameras
# main thing is to store the image we want to use in the frame queue
# Also store the depth info into the depth queue
def rs_camera_handle():
    
    # create the marker object
    marker_inst = marker()

    # start the stream
    while True:
        
        # make sure the pipeline start properly
        try:
            rs_status = marker_inst.get_frames()

            if rs_status:
                
                print ('pipeline started')
                marker_inst.simple_marker_detection()
                marker_inst.single_marker_mask_hist_fillpoly(1)
                
                # clear the queue from previous cycle
                if not frame_queue.empty():
                    frame_queue.get_nowait()

                # put the processed frame into the queue

                frame_queue.put(marker_inst.depth_colormap)
                

        
        except :
            print ('Did not get any frame')

            time.sleep(0.1)



# start the camera frame capture treads
rs_thread = threading.Thread(target=rs_camera_handle, daemon= True)
rs_thread.start()

st.set_page_config(page_title='Underwing Checker V1 Beta' , page_icon='🏗️' , layout='wide')
st.title('Underwing checker')

st.sidebar.title('Navigation')
selection = st.sidebar.radio("Go to" , ['Live Feed' , 'Center Depth'])
frame_place_holder = st.empty()


# Will need to make sure we have what we need for the 
if selection == 'Live Feed':
    
    if not frame_queue.empty():
        image = frame_queue.get()
        frame_place_holder.image(image)
        #live_feed.show(image)
    else:
        st.warning('No image display')

elif selection == 'Center Depth':
    center_depth.show()




#st.page_link('main.py' , label = "Home" , icon = "🏡")
#st.page_link('pages/Depth image stream.py' , label = 'Live stream' , icon = '📺')
#st.page_link('pages/page2.py' , label = 'page2.py' , icon = '🔢' )