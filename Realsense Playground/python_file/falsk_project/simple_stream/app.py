#####################################################
##          Depth camera with flask                ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
# import matplotlib for easy visualization
import matplotlib.pyplot as plt
from flask import Flask, render_template , Response

app = Flask(__name__)

def generate_frame():

    # aruco marker dictionary and intilized
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_detector = cv2.aruco.ArucoDetector(dictionary)


    # # set the ploting style for the ploting 
    # plt.ion()
    # fig, ax = plt.subplots()
    # xdata, ydata = [] , []
    # line, = ax.plot(xdata , ydata , 'r-')
    # ax.set_xlim(0, 100)
    # ax.set_ylim(0, 10000)
    # ax.set_title("Center Depth")
    # ax.set_xlabel("Frame count")
    # ax.set_ylabel("Depth")
    # frame_count = 0

    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # Different resolutions of color and depth streams
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

    # create an marker dict
    marker_dict = {}

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

                # here is the part to append all the infomation in the marker dictionary
                marker_dict[int(m_id)] = avg_depth


                #print('avg_depth is ' , avg_depth)

        ##########################################################################################################################

        # render images:
        # depth align to color on left
        # depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image , alpha = 0.04) , cv2.COLORMAP_JET)
        cv2.aruco.drawDetectedMarkers(depth_colormap , marker_corner , marker_id)

        # add the depth value to the color image 
        #org = (200, 200)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        color = (255, 255, 255)
        thickness = 2
        display_test = "new text \n"

        # create an empty image with only 255 to display all the marker distance information
        id_backgroud = np.zeros((720 , 2560 , 3) , dtype = np.uint8)
        
        
        if marker_dict is not None:
            for i, k in enumerate(marker_dict):
                if k < 30 :
                    display_text = 'ID : ' + str(k) + ' D : ' + str(marker_dict[k]) + "\n"
                    org = (200, 100*(i+1))
                    id_backgroud = cv2.putText(id_backgroud , display_text, org, font, fontScale , color , thickness , cv2.LINE_AA)

        # # here is the part to live plot the depth
        # xdata.append(frame_count)
        # ydata.append(avg_depth)
        # window_size = 100  # Number of data points to display on the plot at any time

        # # Inside the loop, after adding the new data point
        # if len(xdata) > window_size:
        #     ax.set_xlim(xdata[-window_size], xdata[-1])
        # else:
        #     ax.set_xlim(xdata[0], xdata[0] + window_size)   

        # line.set_xdata(xdata)
        # line.set_ydata(ydata)
        # ax.relim()
        # ax.autoscale_view(True, True , True)
        # plt.draw()
        # plt.pause(0.001)
        # frame_count = frame_count + 1


        images = np.hstack((color_image , depth_colormap))
        images = np.vstack((images, id_backgroud))
        images = cv2.resize(images, (1280 , 720) , interpolation= cv2.INTER_LINEAR)


        marker_dict.clear()

        # encode the images to an jpg formate and use yield function to generate an stream
        _ , buffer = cv2.imencode('.jpg' , images)
        image = buffer.tobytes()
        yield(b'--frame\r\n'    
              b'Connect_Type: image/jepg\r\n\r\n' + image + b'\r\n')





@app.route('/')
def index():
    return render_template("index.html")

@app.route('/video_feed')
def video_feed():
    return Response(generate_frame() , mimetype = 'multipart/x-mixed-replace; boundary=frame')

@app.route('/test')
def test():
    return render_template('test.html')

if __name__ == '__main__':
    app.run(debug=True , host = '0.0.0.0' , port = 8080)











