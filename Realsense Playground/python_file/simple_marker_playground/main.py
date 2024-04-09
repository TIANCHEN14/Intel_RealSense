import cv2
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from D405_lib import marker
import streamlit as st

# application class for handling the multithread operation
class Application:
    
    # start the application object, including the marker object
    def __init__(self):
        
        # define marker object
        self.marker = marker()
        self.stop_event = threading.Event()

    # start of the video streams
    def video_stream(self):

        # start the video stream for the intel realsense cameraq
        while not self.stop_event.is_set():

            # getting all the frames from camera
            rs_status = self.marker.get_frames()
            
            if rs_status:

                # run aruco marker detector
                self.marker.simple_marker_detection()
                # doing marker historgram detection
                depth_mask = self.marker.single_marker_mask_hist_fillpoly(1)
                # generate text image
                self.marker.txt_image_generation()

                main_image = np.vstack((self.marker.mask_image , self.marker.text_image))

                cv2.imshow("video stream" , main_image)

            # pressing esc to exit out of the windows
            key = cv2.waitKey(1)

            if key & 0XFF == ord('q') or key == 27:
                self.stop_event.set()
                break


    # thread worker to start the stream
    def start_stream_thread(self):
        threading.Thread(target= self.video_stream, daemon = True).start()

    # update plot function
    def update_plot(self, frame, ax , id):

        # import the depth pixel information
        # hasattr function is here to make sure depth pixel is inside of marker object
        if hasattr(self.marker, 'depth_pixel'): 
            
            if id in self.marker.depth_pixel.keys():
                # import depth pixel
                depth_pixel = self.marker.depth_pixel[id]

                # check make sure it is not an empty list
                if depth_pixel:
                    
                    # clear current plot to not overlay
                    ax.cla()
                    ax.hist(depth_pixel, bins= 200, color = 'blue')
                    ax.set(title = 'Pixel density diagram' , xlabel = 'Depth Value' , ylabel = 'Sample Count')

    # main function to run the project
    def run(self):
        #self.start_stream_thread()
        self.start_stream_thread()
        # setup for live historgram plot
        fig, ax = plt.subplots()
        ani = FuncAnimation(fig, self.update_plot , fargs = (ax, 1), interval = 1000)


        plt.show()
        self.stop_event.set()


if __name__ == "__main__":
    app = Application()
    app.run()