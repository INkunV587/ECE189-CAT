from numba import cuda
from numba import njit
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



@cuda.jit
def gpu_forloop(color_image,depth_image,edge_image,flagresult, n):
	if (flagresult[0] == 1.0):
		pass
	else:
		idx = cuda.threadIdx.x+cuda.blockIdx.x * cuda.blockDim.x
		if(idx < n):
			rt = idx//268
			ct = idx%268
			row=rt+25
			column=ct+25
			if mask[row][column]>100:
				delta_r=-1
				delta_c=-1
				while(delta_r<=1):
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
									bc = framecount
									print(bc)
									flag = 1
								
					
						delta_c=delta_c+1
					delta_c=-1
					delta_r=delta_r+1
				
				flag = 0
				






				if flag == 1:
					flagresult[0] = flag

    
            



if __name__=='__main__':
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
		for item in range(1):
			framecount = framecount + 1
            # Get frameset of color and depth
			frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
			aligned_frames = align.process(frames)

            # Get aligned frames
			aligned_depth_frame = aligned_frames.get_depth_frame() 
# aligned_depth_frame is a 640x480 depth image
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
			flagresult = np.zeros((1))	    

			color_image_device = cuda.to_device(color_image)
			depth_image_device = cuda.to_device(depth_image)
			edge_image_device = cuda.to_device(edge_image)
			flagresult_device = cuda.to_device(flagresult)

			n = 188*268
			gpu_forloop[188, 268](color_image_device,depth_image_device,edge_image_device,flagresult_device, n)
			cuda.synchronize()
            


			cv2.imshow("image",color_image)
			key = cv2.waitKey(1)
			# Press esc or 'q' to close the image window
			if key & 0xFF == ord('q') or key == 27:
				cv2.destroyAllWindows()
				break
	finally:
		pipeline.stop()








