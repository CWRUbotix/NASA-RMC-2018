# coding: utf-8

import numpy as np
import math
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

try:
    	from pylibfreenect2 import OpenCLPacketPipeline
    	pipeline = OpenCLPacketPipeline()
except:
    	try:
       		from pylibfreenect2 import OpenGLPacketPipeline
        	pipeline = OpenGLPacketPipeline()
    	except:
       		from pylibfreenect2 import CpuPacketPipeline
        	pipeline = CpuPacketPipeline()
print("Packet pipeline:", type(pipeline).__name__)

# Create and set logger
logger = createConsoleLogger(LoggerLevel.Debug)
setGlobalLogger(logger)

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    	print("No device connected!")
    	sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

listener = SyncMultiFrameListener(
    FrameType.Color | FrameType.Ir | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()


# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

h,w = 512, 424
FOVX = 1.232202 #horizontal FOV in radians
focal_x = device.getIrCameraParams().fx #focal length x
focal_y = device.getIrCameraParams().fy #focal length y
undistorted = Frame(h, w, 4)
registered = Frame(h, w, 4)

while True:
	frames = listener.waitForNewFrame()
	depth_frame = frames["depth"]

	#convert image
	img = depth_frame.asarray() / 4500.
	imgray = np.uint8(depth_frame.asarray()/255.0)
	#flip images
	img = cv2.flip(img,1)
	imgray = cv2.flip(imgray,1)

	#begin edge detection

	ret,thresh = cv2.threshold(imgray,0,255,cv2.THRESH_BINARY)
	#noise removal
	kernel = np.ones((5,5),np.uint8)
	thresh = cv2.erode(thresh, kernel, iterations = 3)
	thresh = cv2.dilate(thresh, kernel, iterations = 2)
	opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 3)
	#gradient calculation
	gradient = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)
	dilation = cv2.dilate(thresh,kernel,iterations = 1)
	#sure background area
	sure_bg = cv2.erode(opening,kernel,iterations=1)
	dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
	ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
	# Finding unknown region
	sure_fg = np.uint8(sure_fg)
	#finding unknown region
	#unknown = cv2.subtract(gradient,sure_bg)
	unknown = cv2.subtract(sure_bg,sure_fg)
	unkown = cv2.medianBlur(unknown,5)
	img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

	#begin contour detection
	image, contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for cntr in contours:
		try:
			#calculate diamter of equivalent cirlce
			area = cv2.contourArea(cntr)
			equi_diameter = np.sqrt(4*area/np.pi)

			#Hardcoded Diameter Range in pixels
			LOW_DIAMETER_BOUND = 20
			HIGH_DIAMETER_BOUND = 100
			HIGH_DISTANCE_BOUND = 3000
			#Original tolerances were 20 and 150

			if(equi_diameter>LOW_DIAMETER_BOUND and equi_diameter<HIGH_DIAMETER_BOUND): #range needs to be tweaked
				mask = np.zeros(imgray.shape,np.uint8)
				cv2.drawContours(mask,[cntr],0,255,-1)
				pixelpoints = np.transpose(np.nonzero(mask))
				img_fg = cv2.bitwise_and(depth_frame.asarray(),depth_frame.asarray(),mask = mask)
				
				#img_fg = cv2.blur(img_fg,5)
				img_fg = cv2.medianBlur(img_fg,5)

				# Experimenting with different blur settings
				#img_fg = cv2.GaussianBlur(img_fg, (5,5), 0)

				mean_val = cv2.mean(img_fg)
				min_val, distance_to_object, min_loc, max_loc = cv2.minMaxLoc(img_fg)
				
				# Experimenting with object distance
				#distance_to_object = cv2.mean(img_fg)
				#print img_fg[50]

				moment = cv2.moments(cntr)
				cx = int(moment['m10']/moment['m00'])
				cy = int(moment['m01']/moment['m00'])

				# Rule of 57 -- Finding the approximate object diameter
                # ^ Note: This is a really bad way to do it, advise using tangent
                # Due to how Kinect probably returns each pixel distance
				#mm_diameter = (1.0/42.0) * (equi_diameter/6.006) * distance_to_object

                # Preferred Options:
				#mm_diameter = (2 * math.tan((equi_diameter / 2.0 / w) * FOVX) * distance_to_object)
				#(equi_diameter / w) * (2.0 * distance_to_object * math.tan(/2.0)) # ~FOV

				# Unconfirmed: this distance tries to get the unmodified distance
				# Reminder: For this, it's Height x Width
				actualDistmm = depth_frame.asarray()
				actualDistmm = cv2.medianBlur(actualDistmm,5)
				cv2.flip(actualDistmm, 1)
				dist_to_centroid = actualDistmm[cy][cx]

				if dist_to_centroid < HIGH_DISTANCE_BOUND:

					# Sets mm_diameter to the object's diameter in pixels wide, for calibration if desired
					mm_diameter = equi_diameter * (1.0 / focal_x) * dist_to_centroid
					coords = registration.getPointXYZ(depth_frame, cy, cx)

					ellipse = cv2.fitEllipse(cntr)
					img = cv2.ellipse(img,ellipse,(0,255,0),2)
					rect = cv2.minAreaRect(cntr)
					box = cv2.boxPoints(rect)
					box = np.int0(box)
					cv2.drawContours(img,[box],0,(0,0,255),1)

					font = cv2.FONT_HERSHEY_SIMPLEX

					cv2.putText(img, "x" + str(coords[0]), (cx,cy+30), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
					cv2.putText(img, "y" + str(coords[1]), (cx,cy+50), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
					cv2.putText(img, "z" + str(coords[2]), (cx,cy+70), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
					#cv2.putText(img, str(mm_diameter), (cx,cy+20), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)
					
					# Distance to centroid point(?)
					#cv2.putText(img, str(dist_to_centroid), (cx,cy+40), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

		except:
			print "Failed to fit ellipse"


	# NOTE for visualization:
	# cv2.imshow without OpenGL backend seems to be quite slow to draw all
	# things below. Try commenting out some imshow if you don't have a fast
	# visualization backend.

	#cv2.imwrite("thresh.png", thresh)
	#cv2.imwrite("gradient.png", gradient)
	#cv2.imshow("thresh", thresh)
	cv2.imshow("unknown", sure_bg)
	#cv2.imshow("gradient", gradient)
	cv2.imshow("depth", img)
	#break

	listener.release(frames)

	# Use the key 'q' to end!

	key = cv2.waitKey(delay=1)
	if key == ord('q'):
		break

cv2.destroyAllWindows()
device.stop()
device.close()

sys.exit(0)
