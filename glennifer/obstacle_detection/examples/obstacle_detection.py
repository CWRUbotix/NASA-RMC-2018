# coding: utf-8

import numpy as np
import math
import cv2
import pika
import sys
import copy
import time
import messages_pb2
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

# start messaging
exchange_name = 'amq.topic'
connection = pika.BlockingConnection(
    pika.ConnectionParameters(host='localhost'))
channel = connection.channel()
channel.queue_declare(queue='test')
channel.exchange_declare(exchange_name, 'topic', durable=True)


def publish_obstacle_position(x, y, z, diameter):
    msg = messages_pb2.ObstaclePosition()
    msg.x_position = x
    msg.y_position = y
    msg.z_position = z
    msg.diameter = diameter
    print msg
    topic = 'obstacle.position'
    channel.basic_publish(exchange=exchange_name,
                          routing_key=topic,
                          body=msg.SerializeToString())


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
    FrameType.Color | FrameType.Depth)

# Register listeners
device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

device.start()


# NOTE: must be called after device.start()
registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

h, w = 512, 424
FOVX = 1.232202  # horizontal FOV in radians
sending_data = True
window_size = 5
time_delay = 2
focal_x = device.getIrCameraParams().fx  # focal length x
focal_y = device.getIrCameraParams().fy  # focal length y
principal_x = device.getIrCameraParams().cx  # principal point x
principal_y = device.getIrCameraParams().cy  # principal point y
undistorted = Frame(h, w, 4)
registered = Frame(h, w, 4)

while True:
    frames = listener.waitForNewFrame()
    depth_frame = frames["depth"]
    color = frames["color"]
    registration.apply(color, depth_frame, undistorted, registered)
    # convert image
    color = registered.asarray(np.uint8)
    color = cv2.flip(color, 1)
    img = depth_frame.asarray(np.float32) / 4500.
    imgray = np.uint8(depth_frame.asarray(np.float32) / 255.0)
    # flip images
    img = cv2.flip(img, 1)
    imgray = cv2.flip(imgray, 1)

    # begin edge detection
    ret, thresh = cv2.threshold(imgray, 0, 255, cv2.THRESH_BINARY)
    # noise removal
    kernel = np.ones((5, 5), np.uint8)
    thresh = cv2.erode(thresh, kernel, iterations=3)
    thresh = cv2.dilate(thresh, kernel, iterations=2)
    opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=3)
    # gradient calculation
    gradient = cv2.morphologyEx(thresh, cv2.MORPH_GRADIENT, kernel)
    dilation = cv2.dilate(thresh, kernel, iterations=1)
    # sure background area
    sure_bg = cv2.erode(opening, kernel, iterations=1)
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    ret, sure_fg = cv2.threshold(
        dist_transform, 0.7 * dist_transform.max(), 255, 0)
    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    # finding unknown region
    # unknown = cv2.subtract(gradient,sure_bg)
    unknown = cv2.subtract(sure_bg, sure_fg)
    unkown = cv2.medianBlur(unknown, 5)

    # begin contour detection
    image, contours, hierarchy = cv2.findContours(
        sure_bg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    color = cv2.drawContours(color, contours, -1, (0, 255, 0), 1)
    for cntr in contours:
        try:
            # calculate diamter of equivalent cirlce
            area = cv2.contourArea(cntr)
            equi_diameter = np.sqrt(4 * area / np.pi)

            # Hardcoded Diameter Range in pixels
            LOW_DIAMETER_BOUND = 20
            HIGH_DIAMETER_BOUND = 100

            HIGH_DISTANCE_BOUND = 3000
            # Original tolerances were 20 and 150

            # range needs to be tweaked
            if(equi_diameter > LOW_DIAMETER_BOUND and
            	equi_diameter < HIGH_DIAMETER_BOUND):
                mask = np.zeros_like(depth_frame.asarray(np.float32))
                ellipse = cv2.fitEllipse(cntr)
                (circle_x, circle_y), radius = cv2.minEnclosingCircle(cntr)
                equi_diameter = int(radius) * 2
                rect = cv2.minAreaRect(cntr)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                mask = cv2.ellipse(mask, ellipse, (255, 255, 255), -1)
                mask = cv2.erode(mask, (10,10), iterations=5)
                img_fg = cv2.bitwise_and(depth_frame.asarray(np.float32), mask)
                img_fg = cv2.medianBlur(img_fg, 5)
                # Experimenting with different blur settings
                # img_fg = cv2.GaussianBlur(img_fg, (5,5), 0)

                # mean_val = cv2.mean(img_fg)[0] #returns mean value of each channel, we only want first channel
                non_zero_mean = np.mean(img_fg[img_fg.nonzero()])
                mean_val = non_zero_mean
                min_val, distance_to_object, min_loc, max_loc = cv2.minMaxLoc(
                    img_fg)

                moment = cv2.moments(cntr)
                cx = int(moment['m10'] / moment['m00'])
                cy = int(moment['m01'] / moment['m00'])

                # Unconfirmed: this distance tries to get the unmodified distance
                # Reminder: For this, it's Height x Width
                actualDistmm = depth_frame.asarray()
                actualDistmm = cv2.medianBlur(actualDistmm, 5)
                cv2.flip(actualDistmm, 1)
                dist_to_centroid = mean_val  # actualDistmm[cy][cx]

                if dist_to_centroid < HIGH_DISTANCE_BOUND:
                    dist_x = 0.0
                    dist_y = 0.0
                    dist_z = 0.0
                    for i in range(-window_size,window_size):
                        for j in range(-window_size, window_size):
                            coords = registration.getPointXYZ(
                                depth_frame, circle_y + j, circle_x + i)
                            dist_x += coords[0]
                            dist_y += coords[1]
                            dist_z += coords[2]
                    box_size = (2 * window_size) * (2 * window_size)
                    dist_x /= box_size
                    dist_y /= box_size
                    dist_z /= box_size

                    mm_diameter = (equi_diameter) * (1.0 / focal_x) * dist_z
                    #mm_diameter = (math.tan((equi_diameter / 2.0 / w) * FOVX) * coords[2])
                    if(sending_data):
                        if(dist_z < 2.5):
                            publish_obstacle_position(dist_x, dist_y, dist_z, mm_diameter)
                            color = cv2.ellipse(color, ellipse, (0, 255, 0), 2)
                            cv2.drawContours(color, [box], 0, (0, 0, 255), 1)

                            font = cv2.FONT_HERSHEY_SIMPLEX

                            cv2.putText(
                                color, "x " + str(dist_x), (cx, cy + 30), font, 0.4, (0, 0, 255), 1, cv2.LINE_AA)
                            cv2.putText(
                                color, "y " + str(dist_y), (cx, cy + 45), font, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
                            cv2.putText(
                                color, "z " + str(dist_z), (cx, cy + 60), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

                            cv2.putText(color, "diameter = " + str(mm_diameter),
                                        (cx, cy + 15), font, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        except:
            print "Failed to fit ellipse"

    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.

    cv2.imshow("unknown", sure_bg)
    cv2.imshow("depth", color)

    listener.release(frames)

    # Use the key 'q' to end!

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break
    time.sleep(time_delay)

#connection.close()
cv2.destroyAllWindows()
device.stop()
device.close()

sys.exit(0)
