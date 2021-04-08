import multiprocessing as mp
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import matplotlib.pyplot as plt
import pyrealsense2 as rs



def isClose(color_image,depth_image,edge_image,row,column,framecount):
    delta_r=-1
    delta_c=-1
    while (delta_r<=1):
        while (delta_c<=1):
            if (delta_r==0) and (delta_c==0):
                delta_c=delta_c+1
                continue

            point_r=row
            point_c=column

            i=0
            gap=0
            arm=0

            while (i<25):
                i=i+1
                point_c = point_c + delta_c
                point_r = point_r + delta_r
                if (edge_image[point_r][point_c]>200):
                    forward = depth_image[point_r+2*delta_r][point_c+2*delta_c]
                    backward = depth_image[point_r-2*delta_r][point_c-2*delta_c]
                    diff = abs(forward-backward)
                    if (diff>0.15):
                        if gap==0:
                            gap = gap+1
                            arm = backward
                        if gap==1:
                            diff2 = forward - arm
                            if diff2<0.1:
                                bc = str(framecount)
                                print('blocked'+ bc )
                                return True
                                
                    
            delta_c=delta_c+1
        delta_c=-1
        delta_r=delta_r+1
    
    return False


def detection(color_image,depth_image,decf):
    decf.value=0

    color_image=cv2.resize(color_image,(320,240),interpolation = cv2.INTER_AREA)
    depth_image=cv2.resize(depth_image,(320,240),interpolation = cv2.INTER_AREA)

    frame = color_image
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # convert it to uint8 for canny
    blue = color_image[:,:,0]
    blue = np.uint8(blue)

    edge_image = cv2.Canny(blue, 100, 200)
    
    flag = False

    last1_r=-100
    last1_c=-100

    last2_r=-100
    last2_c=-100
    r=30*30
    for rt in range(188):
        for ct in range(268):
            row=rt+25
            column=ct+25
            distance1=(column-last1_c)*(column-last1_c)+(row-last1_r)*(row-last1_r)
            distance2=(column-last2_c)*(column-last2_c)+(row-last2_r)*(row-last2_r)
            if mask[row][column]>100:
                if (distance1>r) and (distance2>r):
                    flag = isClose(color_image,depth_image,edge_image,row,column,1)
                    last2_c=last1_c
                    last2_r=last2_r
                    last1_r=row
                    last1_c=column
            if flag:
                break
        if flag:
            break

    decf.value=1

def tracking(color_image,trcf):
    trcf.value=0
    frame = imutils.resize(color_image, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # plt.imshow(mask)
    # plt.clf()
    # plt.waitforbuttonpress()
    # exit()


    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            start_point = (220,140)
            end_point = (420,340)
            rec_color = (255,0,0)
            rec_thickness = 2
            image = cv2.rectangle(frame, start_point, end_point, rec_color, rec_thickness)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            if (cx < 220):
                send_to_UART(ted, 'L\n')
                print("L")
                frame = cv2.putText(frame, 'left', org, font, fontScale, color, thickness, cv2.LINE_AA) 
                received = read_byte_UART(ted)
                print(received)
                received = read_byte_UART(ted)
                print(received)

            elif cx > 420:
                send_to_UART(ted, "R\n")
                print("R")
                frame = cv2.putText(frame, 'right', org, font, fontScale, color, thickness, cv2.LINE_AA) 
                received = read_byte_UART(ted)
                print(received)
                received = read_byte_UART(ted)
                print(received)
            '''
            if cy < 140:
                send_to_UART(ted, "U\n")
                print("U")
                frame = cv2.putText(frame, 'up', org2, font, fontScale, color, thickness, cv2.LINE_AA) 
                received = read_byte_UART(ted)
                print(received)
                received = read_byte_UART(ted)
                print(received)

            elif cy > 240:
                send_to_UART(ted, "D\n")
                print("D")
                frame = cv2.putText(frame, 'down', org2, font, fontScale, color, thickness, cv2.LINE_AA) 
                received = read_byte_UART(ted)
                print(received)
                received = read_byte_UART(ted)
                print(received)
            '''


    # update the points queue
    pts.appendleft(center)

    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        # thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        # cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)


    trcf.value=1

    
    




if __name__=='__main__':

    # Raspberry Pi UART Init
    ted = init_UART()


    # necessary config for color detection

    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video",
        help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=64,
        help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    # greenLower = (29, 86, 6)
    # greenUpper = (64, 255, 255)
    greenLower = (29, 60, 20)
    greenUpper = (90, 255, 255)
    pts = deque(maxlen=args["buffer"])


    # configs of camera

    # allow the camera or video file to warm up
    time.sleep(2.0)

    # Setup:
    pipeline = rs.pipeline()
    cfg = rs.config()
    # cfg.enable_device_from_file("../object_detection.bag")
    # profile = pipe.start(cfg)

    width = 640
    height = 480
    fps = 30
    framecount = 0


    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(cfg)


    # font 
    font = cv2.FONT_HERSHEY_SIMPLEX 

    # org 
    org = (50, 50) 
    org2 = (50, 50+50) 
    # fontScale 
    fontScale = 1

    # Blue color in BGR 
    color = (255, 0, 0) 

    # Line thickness of 2 px 
    thickness = 2


    # configs to align depth and RGB

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = 1 #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    trcf=mp.Value('i',1)
    decf=mp.Value('i',1)

    try:
        while True:
            framecount = framecount + 1
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

        
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            frame = color_image

            # handle the frame from VideoCapture or VideoStream
            frame = frame[1] if args.get("video", False) else frame

            # if we are viewing a video and we did not grab a frame,
            # then we have reached the end of the video
            if frame is None:
                break

            
            
            if trcf.value==1:
                trc=mp.Process(target=tracking,args=(color_image,trcf))
                trc.start()

            if decf.value==1:
                dec=mp.Process(target=detection,args=(color_image,depth_image,decf))
                dec.start()


            cv2.imshow("image",color_image)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()
        


    