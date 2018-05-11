/**
 * @file april_tags.cpp
 * @brief Example application for April tags library
 * @author: Michael Kaess
 *
 * Opens the first available camera (typically a built in camera in a
 * laptop) and continuously detects April tags in the incoming
 * images. Detections are both visualized in the live image and shown
 * in the text console. Optionally allows selecting of a specific
 * camera in case multiple ones are present and specifying image
 * resolution as long as supported by the camera. Also includes the
 * option to send tag detections via a serial port, for example when
 * running on a Raspberry Pi that is connected to an Arduino.
 */

/**
 * Current Implementation of AprilTags for CWRUbotix NASA RMC 2018, initially version available from https://svn.csail.mit.edu/apriltags
 */

/* Camera Calibration Inputs */
/*double fx = 602.4;
 double fy = 602.4;
 double cx = 319.5;
 double cy = 239.5;
 double k1 = 0.25524;
 F <<
 1, 0,  0,
 double k2 = -10.998;
 double p1 = 0.0;
 double p2 = 0.0;
 double k3 = 125.0015;

 cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
 cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);*/

using namespace std;

#include <iostream>
#include <sstream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <sys/wait.h>

//#include "amqpcpp/AMQPcpp.h"
#include <amqpcpp/AMQPcpp.h>
#include "messages.pb.h"
#include <string>

//#include "utils.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"

// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;

// For Arduino: locally defined serial port access class
#include "Serial.h"

const char* windowName = "Camera 0";
const char* windowName1 = "Camera 1";

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.);
}

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0 * PI;

const char* user = "guest";
const char* pass = "guest";
const char* address = "localhost";

const double greenToCenter = 0.33;
const double yellowToCenter = -0.33;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t + PI, TWOPI) - PI;
	} else {
		t = fmod(t - PI, -TWOPI) + PI;
	}
	return t;
}

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch,
		double& roll) {
	yaw = standardRad(atan2((double) wRo(1, 0), (double) wRo(0, 0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(
			atan2((double) -wRo(2, 0),
					(double) (wRo(0, 0) * c + wRo(1, 0) * s)));
	roll = standardRad(
			atan2((double) (wRo(0, 2) * s - wRo(1, 2) * c),
					(double) (-wRo(0, 1) * s + wRo(1, 1) * c)));
}

//Camera Structure
struct camera {
	double c_fx;	//camera focal length for green cam
	double c_fy;
	double c_px;	//camera principal length for green cam
	double c_py;
	int c_id;
	double c_bearing;
};

//Using the same values to avoid inconsistensies with swapping camera ports
camera greenCam = { 644.12, 644.12, 319.5, 239.5, 2, 0.0 };
camera yellowCam = { 651.12, 651.12, 319.5, 239.5, 0, 0.0 };
camera blueCam = { 644.12, 644.12, 319.5, 239.5, 1, 0.0 };
//camera yellowCam = {538.23, 538.23, 319.5, 239.5, 1};
//camera blueCam = {538.23, 538.23, 319.5, 239.5, 2};

const char* windowGreenCam = "Green Cam";
const char* windowYellowCam = "Yellow Cam";
const char* windowBlueCam = "Blue Cam";

AMQP amqp("guest:guest@localhost");
const char* topic = "locs";
AMQPQueue *queue = amqp.createQueue(topic);

//Initializes AMQP queue etc
void init_Queue() {
	queue->Declare();
	queue->Bind("amq.topic", topic);
	//queue->addEvent(AMQP_MESSAGE, handleReceivedMessage);
	queue->Consume(AMQP_NOACK);
}

class Localization {

	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;

	bool m_draw; // draw image and April tag detections?
	bool m_arduino; // send tag detections to serial port?
	bool m_timing; // print timing information for each tag extraction call

	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame
	double m_fx; // camera focal length in pixels
	double m_fy;
	double m_px; // camera principal point
	double m_py;

	int m_deviceId; // camera id (in case of multiple cameras)

	list<string> m_imgNames;

	cv::VideoCapture m_cap;

	int m_exposure;
	int m_gain;
	int m_brightness;

	Serial m_serial;

	cv::VideoCapture green_cap;
	cv::VideoCapture yellow_cap;
	cv::VideoCapture blue_cap;

public:

	// default constructor
	Localization() :
			// default settings, most can be modified through command line options (see below)
			m_tagDetector(NULL), m_tagCodes(AprilTags::tagCodes36h11),

			m_draw(true), m_arduino(false), m_timing(false),

			m_width(640), m_height(480), m_tagSize(0.165), m_fx(644.12), m_fy(
					644.12), m_px(319.5), m_py(239.5),

			m_exposure(-1), m_gain(-1), m_brightness(-1),

			m_deviceId(0) {
	}

	// changing the tag family
	void setTagCodes(string s) {
		if (s == "16h5") {
			m_tagCodes = AprilTags::tagCodes16h5;
		} else if (s == "25h7") {
			m_tagCodes = AprilTags::tagCodes25h7;
		} else if (s == "25h9") {
			m_tagCodes = AprilTags::tagCodes25h9;
		} else if (s == "36h9") {
			m_tagCodes = AprilTags::tagCodes36h9;
		} else if (s == "36h11") {
			m_tagCodes = AprilTags::tagCodes36h11;
		} else {
			cout << "Invalid tag family specified" << endl;
			exit(1);
		}
	}

	void setup() {
		m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

		// prepare window for drawing the camera images
		if (m_draw) {
			/*cv::namedWindow(windowGreenCam, 1);
			 cv::namedWindow(windowYellowCam, 1);
			 cv::namedWindow(windowBlueCam, 1);*/
		}

		// optional: prepare serial port for communication with Arduino
		if (m_arduino) {
			m_serial.open("/dev/ttyACM0");
		}
	}

	void setupVideo() {

#ifdef EXPOSURE_CONTROL
		// manually setting camera exposure settings; OpenCV/v4l1 doesn't
		// support exposure control; so here we manually use v4l2 before
		// opening the device via OpenCV; confirmed to work with Logitech
		// C270; try exposure=20, gain=100, brightness=150

		string video_str_green = "/dev/video0";
		video_str_green[10] = '0' + greenCam.c_id;
		int deviceGreen = v4l2_open(video_str_green.c_str(),
		O_RDWR | O_NONBLOCK);

		string video_str_yellow = "/dev/video1";
		video_str_yellow[10] = '1' + yellowCam.c_id;
		int deviceYellow = v4l2_open(video_str_yellow.c_str(),
		O_RDWR | O_NONBLOCK);

		string video_str_blue = "/dev/video3";
		video_str_blue[10] = '1' + blueCam.c_id;
		int deviceBlue = v4l2_open(video_str_blue.c_str(), O_RDWR | O_NONBLOCK);

		if (m_exposure >= 0) {
			// not sure why, but v4l2_set_control() does not work for
			// V4L2_CID_EXPOSURE_AUTO...
			struct v4l2_control c_green;
			c_green.id = V4L2_CID_EXPOSURE_AUTO;
			c_green.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
			if (v4l2_ioctl(deviceGreen, VIDIOC_S_CTRL, &c_green) != 0) {
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << m_exposure << endl;
			v4l2_set_control(deviceGreen, V4L2_CID_EXPOSURE_ABSOLUTE,
					m_exposure * 6);
		}
		if (m_gain >= 0) {
			cout << "gain: " << m_gain << endl;
			v4l2_set_control(deviceGreen, V4L2_CID_GAIN, m_gain * 256);
		}
		if (m_brightness >= 0) {
			cout << "brightness: " << m_brightness << endl;
			v4l2_set_control(deviceGreen, V4L2_CID_BRIGHTNESS,
					m_brightness * 256);
		}
		v4l2_close(deviceGreen);

		if (m_exposure >= 0) {
			struct v4l2_control c_yellow;
			c_yellow.id = V4L2_CID_EXPOSURE_AUTO;
			c_yellow.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
			if (v4l2_ioctl(deviceYellow, VIDIOC_S_CTRL, &c_yellow) != 0) {
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << m_exposure << endl;
			v4l2_set_control(deviceYellow, V4L2_CID_EXPOSURE_ABSOLUTE,
					m_exposure * 6);
		}
		if (m_gain >= 0) {
			cout << "gain: " << m_gain << endl;
			v4l2_set_control(deviceYellow, V4L2_CID_GAIN, m_gain * 256);
		}
		if (m_brightness >= 0) {
			cout << "brightness: " << m_brightness << endl;
			v4l2_set_control(deviceYellow, V4L2_CID_BRIGHTNESS,
					m_brightness * 256);
		}
		v4l2_close(deviceYellow);

		if (m_exposure >= 0) {
			struct v4l2_control c_blue;
			c_blue.id = V4L2_CID_EXPOSURE_AUTO;
			c_blue.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
			if (v4l2_ioctl(deviceBlue, VIDIOC_S_CTRL, &c_blue) != 0) {
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << m_exposure << endl;
			v4l2_set_control(deviceBlue, V4L2_CID_EXPOSURE_ABSOLUTE,
					m_exposure * 6);
		}
		if (m_gain >= 0) {
			cout << "gain: " << m_gain << endl;
			v4l2_set_control(deviceBlue, V4L2_CID_GAIN, m_gain * 256);
		}
		if (m_brightness >= 0) {
			cout << "brightness: " << m_brightness << endl;
			v4l2_set_control(deviceBlue, V4L2_CID_BRIGHTNESS,
					m_brightness * 256);
		}
		v4l2_close(deviceBlue);

#endif

		// find and open a USB camera (built in laptop camera, web cam etc)
		green_cap = cv::VideoCapture(greenCam.c_id);
		if (!green_cap.isOpened()) {
			cerr << "ERROR: Can't find Green Camera" << "\n";
			exit(1);
		}

		green_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		green_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

		cout
				<< "Green Camera successfully opened (ignore error messages above...)"
				<< endl;
		cout << "Actual resolution: " << green_cap.get(CV_CAP_PROP_FRAME_WIDTH)
				<< "x" << green_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		yellow_cap = cv::VideoCapture(yellowCam.c_id);
		if (!yellow_cap.isOpened()) {
			cerr << "ERROR: Can't find Yellow Camera" << "\n";
			exit(1);
		}
		yellow_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		yellow_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

		cout
				<< "Yellow Camera successfully opened (ignore error messages above...)"
				<< endl;
		cout << "Actual resolution: " << yellow_cap.get(CV_CAP_PROP_FRAME_WIDTH)
				<< "x" << yellow_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		blue_cap = cv::VideoCapture(blueCam.c_id);
		if (!blue_cap.isOpened()) {
			cerr << "ERROR: Can't find Blue Camera" << "\n";
			exit(1);
		}
		blue_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		blue_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

		cout
				<< "Blue Camera successfully opened (ignore error messages above...)"
				<< endl;
		cout << "Actual resolution: " << blue_cap.get(CV_CAP_PROP_FRAME_WIDTH)
				<< "x" << blue_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

	}

	void print_detection(AprilTags::TagDetection& detection, camera camera,
			string name, double lookie_angle, bool send, int cam_num) const {
		cout << "  Id: " << detection.id << " (Hamming: "
				<< detection.hammingDistance << ")";

		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		Eigen::Vector4d realcoor;
		detection.getRelativeTranslationRotation(m_tagSize, camera.c_fx,
				camera.c_fy, camera.c_px, camera.c_py, translation, rotation,
				realcoor);

		Eigen::Matrix3d F;
		F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
		Eigen::Matrix3d fixed_rot = F * rotation;
		double yaw, pitch, roll;
		wRo_to_euler(fixed_rot, yaw, pitch, roll);

		//x and y coordinates of the camera
		double lookie = lookie_angle * (PI / 180);

		//using x,y camera coords to find coordinates of robot center/wheel center
		double cam_bearing = pitch;
		camera.c_bearing = cam_bearing;
		double cam_x = ((realcoor(0) * cos(cam_bearing))
				- (realcoor(2) * sin(cam_bearing)));
		double cam_y = ((realcoor(0) * sin(cam_bearing))
				+ (realcoor(2) * cos(cam_bearing)));
		double robot_x = cam_x;
		double robot_y = cam_y;

		double robot_bearing = 0;

		if (cam_num == greenCam.c_id) { //green camera
			if (lookie < 0.0) {
				lookie += (2*PI);
			}
			if (lookie < 0) {
				robot_bearing = (lookie - cam_bearing + PI);// % (2 * PI);
			}

			else {
				robot_bearing = (lookie - cam_bearing + PI);// % (2 * PI);
			}

			//cout << "Bearing " << lookie << ", Camera Bearing " << " " << cam_bearing << cout;
			robot_x = -1 * (cam_x + (greenToCenter * cos(robot_bearing)));
			robot_y = cam_y - (greenToCenter * sin(robot_bearing));

			//if (send) {
				AMQPExchange * ex = amqp.createExchange("amq.topic");
				ex->Declare("amq.topic", "topic", AMQP_DURABLE);

				com::cwrubotix::glennifer::LocalizationPosition msg;
				msg.set_x_position((float) robot_x);
				msg.set_y_position((float) robot_y);
				msg.set_bearing_angle((float) robot_bearing);

				int msg_size = msg.ByteSize();
				void *msg_buff = malloc(msg_size);
				msg.SerializeToArray(msg_buff, msg_size);

				ex->Publish((char*) msg_buff, msg_size, "loc.post");

				cout << name << " distance = " << translation.norm()
						<< "m, x coor = " << robot_x //realcoor(0) //(translation.norm() * (sin (pitch)))
						<< ", y coor = " << robot_y //realcoor(2)//(translation.norm() * (cos (pitch)))
						<< ", Robot Bearing = " << (robot_bearing * 180 / PI) << cout ;

				free(msg_buff);
			//}
		}

		if (cam_num == 0) { //yellow camera it is wrong
			if (cam_bearing < 0) {
				robot_bearing = (lookie - cam_bearing + PI);// % (2 * PI);
			}

			else {
				robot_bearing = (lookie - cam_bearing - PI);// % (2 * PI);
			}
			//cout << "Bearing " << lookie << ", Camera Bearing " << cam_bearing
			//		<< cout;
			robot_x = -1 * (cam_x - (greenToCenter * cos(robot_bearing)));
			robot_y = cam_y + (greenToCenter * sin(robot_bearing));

			//if (send) {
				AMQPExchange * ex = amqp.createExchange("amq.topic");
				ex->Declare("amq.topic", "topic", AMQP_DURABLE);

				com::cwrubotix::glennifer::LocalizationPosition msg;
				msg.set_x_position((float) robot_x);
				msg.set_y_position((float) robot_y);
				msg.set_bearing_angle((float) robot_bearing);

				int msg_size = msg.ByteSize();
				void *msg_buff = malloc(msg_size);
				msg.SerializeToArray(msg_buff, msg_size);

				ex->Publish((char*) msg_buff, msg_size, "loc.post");

				cout << name << " distance=" << translation.norm()
						<< "m, x coor = " << robot_x //realcoor(0) //(translation.norm() * (sin (pitch)))
						<< ", y coor = " << robot_y //realcoor(2)//(translation.norm() * (cos (pitch)))
						<< ", Robot Bearing = " << (robot_bearing * (180 / PI)) << cout ;

				free(msg_buff);
			//}
		}

		// Also note that for SLAM/multi-view application it is better to
		// use reprojection error of corner points, because the noise in
		// this relative pose is very non-Gaussian; see iSAM source code
		// for suitable factors.
	}

	bool processImage(cv::Mat& image, cv::Mat& image_gray, camera camera,
			string name, double angle, bool send, int cam_num) {
		// alternative way is to grab, then retrieve; allows for
		// multiple grab when processing below frame rate - v4l keeps a
		// number of frames buffered, which can lead to significant lag
		//      m_cap.grab();
		//      m_cap.retrieve(image);

		// detect April tags (requires a gray scale image)
		bool tag_Detected = false;

		cv::cvtColor(image, image_gray, CV_BGR2GRAY);
		double t0;
		if (m_timing) {
			t0 = tic();
		}
		vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(
				image_gray);
		if (m_timing) {
			double dt = tic() - t0;
			cout << "Extracting tags took " << dt << " seconds." << endl;
		}

		if (detections.size() > 0) {
			tag_Detected = true;
		}
		// print out each detection
		cout << detections.size() << " tags detected:" << endl;
		for (int i = 0; i < detections.size(); i++) {
			print_detection(detections[i], camera, name, angle, send, cam_num);
		}

		// show the current image including any detections
		if (m_draw) {
			for (int i = 0; i < detections.size(); i++) {
				// also highlight in the image
				detections[i].draw(image);
			}
			//imshow(name, image); // OpenCV call
			/*const char* frame;
			 std::vector<uchar> buffer;
			 imencode(".jpg", image, buffer);
			 AMQPExchange * ex = amqp.createExchange("amq.topic");
			 ex->Declare("amq.topic", "topic", AMQP_DURABLE);
			 int msg_size = sizeof(buffer);
			 void *msg_buff = malloc(msg_size);
			 //buffer.SerializeToArray(msg_buff, msg_size);

			 ex->Publish(, msg_size, name);*/
		}

		// optionally send tag information to serial port (e.g. to Arduino)
		if (m_arduino) {
			if (detections.size() > 0) {
				// only the first detected tag is sent out for now
				Eigen::Vector3d translation;
				Eigen::Matrix3d rotation;
				Eigen::Vector4d realcoor;
				detections[0].getRelativeTranslationRotation(m_tagSize,
						camera.c_fx, camera.c_fy, camera.c_px, camera.c_py,
						translation, rotation, realcoor);
				m_serial.print(detections[0].id);
				m_serial.print(",");
				m_serial.print(translation(0));
				m_serial.print(",");
				m_serial.print(translation(1));
				m_serial.print(",");
				m_serial.print(translation(2));

				m_serial.print("\n");
			} else {
				// no tag detected: tag ID = -1
				m_serial.print("-1,0.0,0.0,0.0\n");
			}
		}

		return tag_Detected;
	}

	void handleLookie(double angle, string key) {
		AMQPExchange * ex = amqp.createExchange("amq.topic");
		ex->Declare("amq.topic", "topic", AMQP_DURABLE);
		//
		com::cwrubotix::glennifer::PositionControlCommand msg;
		msg.set_position((float) angle);
		msg.set_timeout(0.0F);

		int msg_size = msg.ByteSize();
		void *msg_buff = malloc(msg_size);
		msg.SerializeToArray(msg_buff, msg_size);
		ex->Publish((char*) msg_buff, msg_size, key);
	}

	// Load and process a single image
	void loadImages() {
		cv::Mat image;
		cv::Mat image_gray;

		/*for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
		 image = cv::imread(*it); // load image with opencv
		 processImage(image, image_gray);
		 while (cv::waitKey(100) == -1) {}
		 }*/
	}

	// Video or image processing?
	bool isVideo() {
		return m_imgNames.empty();
	}

	// The processing loop where images are retrieved, tags detected,
	// and information about detections generated
	void loop() {

		cv::Mat green_image;
		cv::Mat green_image_gray;
		cv::Mat yellow_image;
		cv::Mat yellow_image_gray;
		cv::Mat blue_image;
		cv::Mat blue_image_gray;

		//are we seeing a tag
		bool green_detect = false;
		bool yellow_detect = false;
		bool blue_detect = false;

		//lookie angle
		double green_lookie = 90.0;
		double yellow_lookie = 270.0;

		//sweep to the "right" with both cameras
		bool green_sweep_right = true;
		bool yellow_sweep_right = true;

		//last camera bearing
		double last_green = 0.0;
		double last_yellow = 0.0;

		handleLookie(green_lookie, "motorcontrol.looky.turn.right");
		handleLookie(yellow_lookie, "motorcontrol.looky.turn.left");

		double sleep_time = 0.1;

		int frame = 0;
		double last_t = tic();
		while (true) {

			// capture frame
			green_cap >> green_image;
			cout << "communicating green cam data..." << endl;
			yellow_cap >> yellow_image;
			cout << "communicating yellow cam data..." << endl;
			blue_cap >> blue_image;
			cout << "communicating blue data..." << endl;

			//first measurement to see if we see a tag at all

			green_detect = processImage(green_image, green_image_gray, greenCam,
					windowGreenCam, green_lookie, false, greenCam.c_id);
			yellow_detect = processImage(yellow_image, yellow_image_gray,
					yellowCam, windowYellowCam, yellow_lookie, false,
					yellowCam.c_id);
			blue_detect = processImage(blue_image, blue_image_gray, blueCam,
					windowBlueCam, 0, false, blueCam.c_id);

			if (!green_detect) {
				if (green_lookie < 250.0 && green_sweep_right) {
					//sleep_time = 0.10;
					green_lookie += 2;
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
					//sleep(sleep_time);
				}
				if (green_lookie >= 250.0) {
					//sleep_time = 0.10;
					green_sweep_right = false;
					printf("green sweep false");
					green_lookie -= 2;
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
					//sleep(sleep_time);
				}
				if (green_lookie > -70.0 && !green_sweep_right) {
					//sleep_time = 0.10;
					green_lookie -= 2;
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
					//sleep(sleep_time);
				}
				if (green_lookie <= -70.0) {
					//sleep_time = 0.10;
					//printf("green sweep true");
					green_sweep_right = true;
					green_lookie += 2;
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
					//sleep(sleep_time);
				}

			}

			if (green_detect) {
				green_detect = processImage(green_image, green_image_gray,
										greenCam, windowGreenCam, green_lookie, true,
										greenCam.c_id);
				//printf("Green Camera Tag Detected");
			}
			//second measurement in case we see the tag if we need to move to track the tag
			/*if (green_detect && last_green < greenCam.c_bearing + 10
					&& last_green > greenCam.c_bearing - 10) {
				last_green = greenCam.c_bearing;
				green_detect = processImage(green_image, green_image_gray,
						greenCam, windowGreenCam, green_lookie, true,
						greenCam.c_id);
			}
			if (green_detect && last_green < greenCam.c_bearing + 10
					&& last_green > greenCam.c_bearing - 10) {
				if (last_green > greenCam.c_bearing + 10) {
					green_lookie -= (last_green - greenCam.c_bearing);
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
				} else {
					green_lookie += (greenCam.c_bearing - last_green);
					handleLookie(green_lookie, "motorcontrol.looky.turn.right");
				}
				last_green = greenCam.c_bearing;
				green_detect = processImage(green_image, green_image_gray,
						greenCam, windowGreenCam, green_lookie, true,
						greenCam.c_id);
			}*/

			if (!yellow_detect) {
				if (yellow_lookie > 110.0 && yellow_sweep_right) {
					//sleep_time = 0.10;
					yellow_lookie -= 2;
					handleLookie(yellow_lookie, "motorcontrol.looky.turn.left");
					//sleep(sleep_time);
				}
				if (yellow_lookie <= 110.0) {
					//sleep_time = 0.10;
					yellow_sweep_right = false;
					yellow_lookie += 2;
					handleLookie(yellow_lookie, "motorcontrol.looky.turn.left");
					//sleep(sleep_time);
				}
				if (yellow_lookie < 430.0 && !yellow_sweep_right) {
					//sleep_time = 0.10;
					//increment by one
					yellow_lookie += 2;
					handleLookie(yellow_lookie, "motorcontrol.looky.turn.left");
					//sleep(sleep_time);
				}
				if (yellow_lookie >= 430.0) {
					//sleep_time = 0.10;
					//printf("green sweep true");
					yellow_sweep_right = true;
					yellow_lookie -= 2;
					handleLookie(yellow_lookie, "motorcontrol.looky.turn.left");
					//sleep(sleep_time);
				}
			}

			if (yellow_detect) {
				yellow_detect = processImage(yellow_image, yellow_image_gray,
						yellowCam, windowYellowCam, yellow_lookie, true,
						yellowCam.c_id);
				///printf("Yellow Camera Tag Detected");
			}

			// print out the frame rate at which image frames are being processed
			frame++;
			if (frame % 10 == 0) {
				double t = tic();
				cout << "  " << 10. / (t - last_t) << " fps" << endl;
				last_t = t;
			}

			// exit if any key is pressed
			if (cv::waitKey(1) >= 0)
				break;
			sleep(sleep_time);

		}
	}

};
// Demo

// here is were everything begins
int main(int argc, char* argv[]) {
	Localization localization;

	// process command line options
	//demo.parseOptions(argc, argv);

	localization.setup();

	if (localization.isVideo()) {
		cout << "Processing video" << endl;

		// setup image source, window for drawing, serial port...
		localization.setupVideo();

		// the actual processing loop where tags are detected and visualized
		localization.loop();

	} else {
		cout << "Processing image" << endl;

		// process single image
		localization.loadImages();
	}

	return 0;
}

