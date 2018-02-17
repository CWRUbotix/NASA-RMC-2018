
import numpy as np
import cv2
import sys
import copy
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame
from pylibfreenect2 import createConsoleLogger, setGlobalLogger
from pylibfreenect2 import LoggerLevel

try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    try:
        from pylibfreenect2 import OpenCLPacketPipeline
        pipeline = OpenCLPacketPipeline()
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

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

# Optinal parameters for registration
# set True if you need
need_bigdepth = False
need_color_depth_map = False
w,h = 424, 512

bigdepth = Frame(1920, 1082, 4) if need_bigdepth else None
color_depth_map = np.zeros((424, 512),  np.int32).ravel() \
    if need_color_depth_map else None

while True:
    frames = listener.waitForNewFrame()

    color = frames["color"]
    ir = frames["ir"]
    depth = frames["depth"]

    registration.apply(color, depth, undistorted, registered,
                       bigdepth=bigdepth,
                       color_depth_map=color_depth_map)

    # Test getting distance at the center pixel
    test_get_dist_center = copy.copy(depth.asarray())
    # Makes a fresh version of depth.asarray
    depthArray = copy.copy(depth.asarray())

    cv2.flip(test_get_dist_center, 1)
    cv2.flip(depthArray, 1)
    dist = test_get_dist_center[w/2][h/2]

    # Using a box of some amount to bound with
    box_size = 8
    rect = test_get_dist_center[w/2-box_size:w/2+box_size , h/2-box_size:h/2+box_size]
    dist = np.median(rect)
    #dist = test_get_dist_center[256][212]

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(test_get_dist_center, str(dist), (h/2,w/2), font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

    # Attempting to try Canny/Etc Filtering
    imgray8bit = np.uint8(depthArray/255)

    imgray = depthArray/4500

    # Preprocess with a Gaussian Blur?
    imgFilterGauss = cv2.GaussianBlur(imgray8bit, (5,5), 0)
    imgFilterGauss3 = cv2.GaussianBlur(imgray8bit, (3,3), 0)

    kernel = np.ones((5,5),np.uint8)
    threshDilate = cv2.dilate(imgFilterGauss3, kernel, iterations = 1)
    thresh = cv2.erode(threshDilate, kernel, iterations=1)

    #thresh = cv2.dilate(thresh, kernel, iterations = 1)
    #thresh = cv2.erode(threshDilate, kernel, iterations=1)
    #what to display
    #imgFilter = thresh
    imgFilter = cv2.Canny(thresh, 5, 15)

    # NOTE for visualization:
    # cv2.imshow without OpenGL backend seems to be quite slow to draw all
    # things below. Try commenting out some imshow if you don't have a fast
    # visualization backend.
    #cv2.imshow("ir", ir.asarray() / 65535.)

    #cv2.imshow("depth", depth.asarray() / 4500.)

    # How to see where we're getting info at:
    #test_get_dist_center = cv2.circle(test_get_dist_center,(w/2,h/2), 2, (0,0,255), -1)
    
    #lol, so apparantly this is supposed to be h by w???
    test_get_dist_center = cv2.circle(test_get_dist_center,(h/2,w/2), 2, (0,0,255), -1)

    cv2.imshow("depth", test_get_dist_center / 4500.)
    #cv2.imshow("object detection filter Gauss 11", imgFilterGauss11)
    cv2.imshow("object detection filter", imgFilter)
    #cv2.imshow("color", cv2.resize(color.asarray(), (int(1920 / 3), int(1080 / 3))))
    #cv2.imshow("registered", registered.asarray(np.uint8))

    if need_bigdepth:
        cv2.imshow("bigdepth", cv2.resize(bigdepth.asarray(np.float32),
                                          (int(1920 / 3), int(1082 / 3))))
    if need_color_depth_map:
        cv2.imshow("color_depth_map", color_depth_map.reshape(424, 512))

    listener.release(frames)

    key = cv2.waitKey(delay=1)
    if key == ord('q'):
        break

device.stop()
device.close()

sys.exit(0)