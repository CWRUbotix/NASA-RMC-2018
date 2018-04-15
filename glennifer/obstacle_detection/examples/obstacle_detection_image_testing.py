# coding: utf-8

import numpy as np
import math
import time
import cv2
import sys
import os
import glob

h,w = 512, 424
FOVX = 1.232202 #horizontal FOV in radians
frame_i = 0

while True:

<<<<<<< HEAD
	img = np.load("test_frames/" + str(frame_i) +".npy") / 4500.
	imgray = np.uint8(img/255.0)
	cv2.imshow("depth", img)
	#save depth image as raw frame in video
	#out.write(img)
=======
	img = cv2.imread("test_frames/frame_" + str(frame_i) +".bmp") #* 4500.0
	#imgray = np.uint8(img/255.0)
	imgray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
	cv2.imshow("depth", img)
>>>>>>> ebe939e5b60ab866f095151055c59ae2f96005a4

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
	#img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

	#begin contour detection
	image, contours, hierarchy = cv2.findContours(sure_bg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#color = cv2.drawContours(color, contours, -1, (0,255,0), 1)
	for cntr in contours:
		#try:
			#calculate diamter of equivalent cirlce
			area = cv2.contourArea(cntr)
			equi_diameter = np.sqrt(4*area/np.pi)

			#Hardcoded Diameter Range in pixels
			LOW_DIAMETER_BOUND = 20
			HIGH_DIAMETER_BOUND = 100

			HIGH_DISTANCE_BOUND = 3000
			#Original tolerances were 20 and 150

			if(equi_diameter>LOW_DIAMETER_BOUND and equi_diameter<HIGH_DIAMETER_BOUND): #range needs to be tweaked
				mask = np.zeros_like(img * 4500.)
				ellipse = cv2.fitEllipse(cntr)
				(x,y),radius = cv2.minEnclosingCircle(cntr)
				equi_diameter = int(radius) * 2
				rect = cv2.minAreaRect(cntr)
				box = cv2.boxPoints(rect)
				box = np.int0(box)
				mask = cv2.ellipse(mask,ellipse,(255,255,255),-1)
				mask = cv2.erode(mask, kernel, iterations=5)
				img_fg = cv2.bitwise_and(img,mask)
				img_fg = cv2.medianBlur(img_fg,5)



				# Experimenting with different blur settings
				#img_fg = cv2.GaussianBlur(img_fg, (5,5), 0)

				#mean_val = cv2.mean(img_fg)[0] #returns mean value of each channel, we only want first channel
				non_zero_mean = np.median(img_fg[img_fg.nonzero()])
				mean_val = non_zero_mean
				min_val, distance_to_object, min_loc, max_loc = cv2.minMaxLoc(img_fg)

				# Experimenting with object distance
				#distance_to_object = cv2.mean(img_fg)
				#print img_fg[50]

				moment = cv2.moments(cntr)
				cx = int(moment['m10']/moment['m00'])
				cy = int(moment['m01']/moment['m00'])

				# Preferred Options:
				#mm_diameter = (2 * math.tan((equi_diameter / 2.0 / w) * FOVX) * distance_to_object)
				#(equi_diameter / w) * (2.0 * distance_to_object * math.tan(/2.0)) # ~FOV

				# Unconfirmed: this distance tries to get the unmodified distance
				# Reminder: For this, it's Height x Width
				actualDistmm = img * 4500.
				actualDistmm = cv2.medianBlur(actualDistmm,5)
				cv2.flip(actualDistmm, 1)
				dist_to_centroid = mean_val #actualDistmm[cy][cx]

				if dist_to_centroid < HIGH_DISTANCE_BOUND:
					coords = registration.getPointXYZ(undistorted, max_loc[1], max_loc[0])
					mm_diameter = (equi_diameter) * (1.0 / focal_x) * coords[2]

					publish_obstacle_position(coords[0],coords[1],coords[2],mm_diameter)

					#color = cv2.ellipse(color,ellipse,(0,255,0),2)
					#cv2.drawContours(color,[box],0,(0,0,255),1)

					font = cv2.FONT_HERSHEY_SIMPLEX

					cv2.putText(img, "x" + str(coords[0]), (cx,cy+30), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
					cv2.putText(img, "y" + str(coords[1]), (cx,cy+45), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
					cv2.putText(img, "z" + str(coords[2]), (cx,cy+60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

					cv2.putText(img,"diameter = " + str(mm_diameter), (cx,cy + 15), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

		#except:
			#print "Failed to fit ellipse"


	# NOTE for visualization:
	# cv2.imshow without OpenGL backend seems to be quite slow to draw all
	# things below. Try commenting out some imshow if you don't have a fast
	# visualization backend.

	#cv2.imshow("unknown", sure_bg)
	cv2.imshow("depth", img)

<<<<<<< HEAD

	time.sleep(1)
=======
	listener.release(frames)
	# Use the key 'q' to end!

>>>>>>> ebe939e5b60ab866f095151055c59ae2f96005a4
	key = cv2.waitKey(delay=1)
	if key == ord('q'):
		break
	#wait to process the next frame
	time.sleep(1)

cv2.destroyAllWindows()
device.stop()
device.close()

sys.exit(0)
