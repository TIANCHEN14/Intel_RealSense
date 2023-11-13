# this is the tutorial for the intel realsense D435 camrea
# this is the first program that even writen by me 

import pyrealsense2 as rs

try:
    # create an context object. This object owns the handle to all connected real sense camera
    pipeline = rs.pipeline()

    # configure the stream
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480 , rs.format.z16, 30)

    # start streaming
    pipeline.start(config)

    while True:
        # this call waits until a new coherent set if frames is available on a device
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth: contiune

        # print an simple text base respresnetion of the image

        converage = [0]*64
        for y in range(480):
            for x in range(640):
                dist = depth.get_distance(x,y)
                print(dist)
                if 0 < dist and dist < 1:
                    converage[x//10] += 1



            # if y%20 == 19:
            #     line = ''
            #     for c in converage:
            #         line += " .:nhBXWW"[c//25]

            #     coverage = [0]*64
            #     print(line)

    exit(0)

except Exception as e:
    print(e)
    pass