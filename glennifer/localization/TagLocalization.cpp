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
 * Current Implementation of AprilTags for CWRUbotix NASA RMC 2018
 */


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
//#include "messages.pb.h"

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

//For vector cache
int cacheCameraIndex = -1;

Eigen::Vector3d c0CacheVector0;
Eigen::Vector3d c0CacheVector1;

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

const char* user = "guest";
const char* pass = "guest";
const char* address = "localhost";

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
	if (t >= 0.) {
		t = fmod(t+PI, TWOPI) - PI;
	} else {
		t = fmod(t-PI, -TWOPI) + PI;
	}
	return t;
}

//#include "CamCalibration.cpp"

/* Camera Calibration Inputs */
/*double fx = 602.4;
double fy = 602.4;
double cx = 319.5;
double cy = 239.5;
double k1 = 0.25524;
double k2 = -10.998;
double p1 = 0.0;
double p2 = 0.0;
double k3 = 125.0015;

cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << k1, k2, p1, p2, k3);*/

/**
 * Convert rotation matrix to Euler angles
 */
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
	yaw = standardRad(atan2((double)wRo(1,0), (double)wRo(0,0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2((double)-wRo(2,0), (double)(wRo(0,0)*c + wRo(1,0)*s)));
	roll  = standardRad(atan2((double)(wRo(0,2)*s - wRo(1,2)*c), (double)(-wRo(0,1)*s + wRo(1,1)*c)));
}


AMQP amqp("guest:guest@localhost");
const char* topic = "locs";
AMQPQueue *queue = amqp.createQueue(topic);

//this should be set in methods or somewhere appropriate
//all is here for ease of access during testing

//similar code to ConsumerThread::run()
//so, should check dor fuplicate topic threads???

//Initializes AMQP queue etc
void init_Queue() {
	queue->Declare();
	queue->Bind("amq.topic", topic);
	//queue->addEvent(AMQP_MESSAGE, handleReceivedMessage);
	queue->Consume(AMQP_NOACK);
}

class Demo {

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
	cv::VideoCapture m_cap1;

	/*cv::VideoCapture m_cap2;*/

	int m_exposure;
	int m_gain;
	int m_brightness;

	Serial m_serial;

	bool procPort = false;
	bool procStarboard = false;
	bool procStern = false;

public:

	// default constructor
	Demo() :
		// default settings, most can be modified through command line options (see below)
		m_tagDetector(NULL),
		m_tagCodes(AprilTags::tagCodes36h11),

		m_draw(true),
		m_arduino(false),
		m_timing(false),

		m_width(640), //640
		m_height(480), //480
		m_tagSize(0.165), //0.166
		m_fx(644.12),	//600 approximate, 644.12 second good calibration
		m_fy(644.12),	//600 approximate, 644.12 second good calibration
		m_px(319.5),	//m_width/2  //same 399.5 //319.5
		m_py(239.5),	//m_height/2  //with these values 299.5 //239.5

		m_exposure(-1),
		m_gain(-1),
		m_brightness(-1),

		m_deviceId(0) //0
{}

	// changing the tag family
	void setTagCodes(string s) {
		if (s=="16h5") {
			m_tagCodes = AprilTags::tagCodes16h5;
		} else if (s=="25h7") {
			m_tagCodes = AprilTags::tagCodes25h7;
		} else if (s=="25h9") {
			m_tagCodes = AprilTags::tagCodes25h9;
		} else if (s=="36h9") {
			m_tagCodes = AprilTags::tagCodes36h9;
		} else if (s=="36h11") {
			m_tagCodes = AprilTags::tagCodes36h11;
		} else {
			cout << "Invalid tag family specified" << endl;
			exit(1);
		}
	}

	// parse command line options to change default behavior

	void setup() {
		m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

		// prepare window for drawing the camera images
		if (m_draw) {
			cv::namedWindow(windowName, 1);
		}

		// optional: prepare serial port for communication with Arduino
		if (m_arduino) {
			m_serial.open("/dev/ttyACM0");
		}
	}

	void setupVideo() {


		AMQP amqp("guest:guest@localhost");

		//AMQPExchange * ex = amqp.createExchange("amqp.topic");
		//ex->Declare("amqp.topic", "topic", AMQP_DURABLE);

		//AMQPQueue * qu2 = amqp.createQueue("q2");
		//qu2->Declare();
		//qu2->Bind( "e", "");

#ifdef EXPOSURE_CONTROL
		// manually setting camera exposure settings; OpenCV/v4l1 doesn't
		// support exposure control; so here we manually use v4l2 before
		// opening the device via OpenCV; confirmed to work with Logitech
		// C270; try exposure=20, gain=100, brightness=150

		string video_str = "/dev/video0";
		video_str[10] = '0' + m_deviceId;
		int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

		//set up other cameras
		string video_str1 = "/dev/video1";
		video_str1[10] = '1' + m_deviceId;
		int device1 = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

		//string video_str2 = "/dev/video2";
		//video_str2[10] = '2' + m_deviceId;
		//int device2 = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

		//camera 1
		if (m_exposure >= 0) {
			// not sure why, but v4l2_set_control() does not work for
			// V4L2_CID_EXPOSURE_AUTO...
			struct v4l2_control c;
			c.id = V4L2_CID_EXPOSURE_AUTO;
			c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
			if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << m_exposure << endl;
			v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
		}
		if (m_gain >= 0) {
			cout << "gain: " << m_gain << endl;
			v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
		}
		if (m_brightness >= 0) {
			cout << "brightness: " << m_brightness << endl;
			v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
		}
		v4l2_close(device);

		//camera 2
		if (m_exposure >= 0) {
			struct v4l2_control c1;
			c1.id = V4L2_CID_EXPOSURE_AUTO;
			c1.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
			if (v4l2_ioctl(device1, VIDIOC_S_CTRL, &c1) != 0) {
				cout << "Failed to set... " << strerror(errno) << endl;
			}
			cout << "exposure: " << m_exposure << endl;
			v4l2_set_control(device1, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
		}
		if (m_gain >= 0) {
			cout << "gain: " << m_gain << endl;
			v4l2_set_control(device1, V4L2_CID_GAIN, m_gain*256);
		}
		if (m_brightness >= 0) {
			cout << "brightness: " << m_brightness << endl;
			v4l2_set_control(device1, V4L2_CID_BRIGHTNESS, m_brightness*256);
		}
		v4l2_close(device1);

		//camera 3
		/*if (m_exposure >= 0) {
				struct v4l2_control c2;
				c2.id = V4L2_CID_EXPOSURE_AUTO;
				c2.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
				if (v4l2_ioctl(device2, VIDIOC_S_CTRL, &c2) != 0) {
				  cout << "Failed to set... " << strerror(errno) << endl;
				}
				cout << "exposure: " << m_exposure << endl;
				v4l2_set_control(device2, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
			}
			if (m_gain >= 0) {
					cout << "gain: " << m_gain << endl;
					v4l2_set_control(device2, V4L2_CID_GAIN, m_gain*256);
			}
			if (m_brightness >= 0) {
					cout << "brightness: " << m_brightness << endl;
					v4l2_set_control(device2, V4L2_CID_BRIGHTNESS, m_brightness*256);
			}
			v4l2_close(device2);*/

#endif

		// find and open a USB camera (built in laptop camera, web cam etc)
		m_cap = cv::VideoCapture(m_deviceId);
		if(!m_cap.isOpened()) {
			cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
			exit(1);
		}
		m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
		//m_cap.set(CV_CAP_PROP_FPS, 5);
		cout << "Camera successfully opened (ignore error messages above...)" << endl;
		cout << "Actual resolution: "
				<< m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
				<< m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		//TODO: for other camera
		//set up other cameras
		m_cap1 = cv::VideoCapture(1);
		if(!m_cap1.isOpened()) {
			cerr << "ERROR: Can't find video device " << 1 << "\n"; //change to deviceId_One if possible
			exit(1);
		}
		m_cap1.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		m_cap1.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
		//m_cap1.set(CV_CAP_PROP_FPS, 5);
		cout << "Camera successfully opened (ignore error messages above...)" << endl;
		cout << "Actual resolution: "
				<< m_cap1.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
				<< m_cap1.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		/*m_cap2 = cv::VideoCapture(2);
            if(!m_cap2.isOpened()) {
          cerr << "ERROR: Can't find video device " << 2 );<< "\n";
          exit(1);
        }
        m_cap2.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
        m_cap2.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
        cout << "Camera successfully opened (ignore error messages above...)" << endl;
        cout << "Actual resolution: "
             << m_cap2.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
             << m_cap2.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;*/

	}


	void print_detection(AprilTags::TagDetection& detection) const {
		cout << "\t\t  Id: " << detection.id
				<< " (Hamming: " << detection.hammingDistance << ")";

		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
				translation, rotation);

		//calculating rotation matric
		Eigen::Matrix3d F;
		F <<
				1, 0,  0,
				0,  -1,  0,
				0,  0,  1;
		Eigen::Matrix3d fixed_rot = F*rotation;
		double yaw, pitch, roll;
		wRo_to_euler(fixed_rot, yaw, pitch, roll);

		//Outputting the vector components to the AprilTag
		cout << "  distance=" << translation.norm()
        				 << "m, x=" << translation(0)
						 << ", y=" << translation(1)
						 << ", z=" << translation(2)
						 << ", yaw(x)=" << yaw
						 << ", pitch(z)=" << pitch
						 << ", roll(y)=" << roll;
		cout   << endl; //added ; cout to fix eclipse bug

		if(cacheCameraIndex == 0){
			if(detection.id == 1){
				//TODO: see if garbage collection affects this. hopyfully not
				c0CacheVector0 = translation;
			}else if(detection.id == 3){
				c0CacheVector1 = translation;
			}
		}else if(cacheCameraIndex == 1){
			//TODO: this
		}

		//AMQP

		//AMQPExchange * ex = amqp.createExchange("amq.topic");
		//ex->Declare("amq.topic", "topic", AMQP_DURABLE);
		//char msg [50];
		//sprintf(msg,"%d",(int)translation.norm());
		//ex->Publish(msg, sizeof(msg), "localization");


		// Also note that for SLAM/multi-view application it is better to
		// use reprojection error of corner points, because the noise in
		// this relative pose is very non-Gaussian; see iSAM source code
		// for suitable factors.
	}

	//From wikipedia, this is for testing. again, ask chad
	//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	Eigen::Quaterniond toQuaternion(double pitch, double roll, double yaw)
	{
		Eigen::Quaterniond q;
		// Abbreviations for the various angular functions
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);

		q.w() = cy * cr * cp + sy * sr * sp;
		q.x() = cy * sr * cp - sy * cr * sp;
		q.y() = cy * cr * sp + sy * sr * cp;
		q.z() = sy * cr * cp - cy * sr * sp;
		return q;
	}

	bool processImage(cv::Mat& image, cv::Mat& image_gray) {
		// alternative way is to grab, then retrieve; allows for
		// multiple grab when processing below frame rate - v4l keeps a
		// number of frames buffered, which can lead to significant lag
		//      m_cap.grab();
		//      m_cap.retrieve(image);

		// detect April tags (requires a gray scale image)
		cv::Mat temp = image.clone();
		//cv::undistort(temp, image,cameraMatrix, distortionCoefficients);
		cv::cvtColor(image, image_gray, CV_BGR2GRAY);
		double t0;
		if (m_timing) {
			t0 = tic();
		}
		vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
		if (m_timing) {
			double dt = tic()-t0;
			cout << "Extracting tags took " << dt << " seconds." << endl;
		}

		// print out each detection
		cout << "\t" << detections.size() << " Camera tags detected:" << endl;
		bool ret = false;
		cout << "\tdetections.size: "<< detections.size() << endl;
		for (int i=0; i<detections.size(); i++){
			print_detection(detections[i]); //previously commented out
			ret = true;
		}

		// show the current image including any detections
		if (m_draw) {
			for (int i=0; i<detections.size(); i++) {
				// also highlight in the image
				detections[i].draw(image);
			}
			imshow(windowName, image); // OpenCV call
		}

		//TODO: un-comment. commented for better runtime on chad's virtual machine
		/*		// optionally send tag information to serial port (e.g. to Arduino)
		if (m_arduino) {
			if (detections.size() > 0) {
				// only the first detected tag is sent out for now
				Eigen::Vector3d translation;
				Eigen::Matrix3d rotation;
				detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
						translation, rotation);
				m_serial.print(detections[0].id);
				m_serial.print(",");
				// m_serial.print(translation(0));
				m_serial.print(",");
				//m_serial.print(translation(1));
				m_serial.print(",");
				// m_serial.print(translation(2));
				m_serial.print("\n");
			} else {
				// no tag detected: tag ID = -1
				m_serial.print("-1,0.0,0.0,0.0\n");
			}
		}*/


		return ret;
	}

	/*
	void processImage2(cv::Mat& image, cv::Mat& image_gray) {
		// alternative way is to grab, then retrieve; allows for
		// multiple grab when processing below frame rate - v4l keeps a
		// number of frames buffered, which can lead to significant lag
		//      m_cap.grab();
		//      m_cap.retrieve(image);

		// detect April tags (requires a gray scale image)
		cv::Mat temp = image.clone();
		//cv::undistort(temp, image,cameraMatrix, distortionCoefficients);
		cv::cvtColor(image, image_gray, CV_BGR2GRAY);
		double t0;
		if (m_timing) {
			t0 = tic();
		}
		vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
		if (m_timing) {
			double dt = tic()-t0;
			cout << "Extracting tags took " << dt << " seconds." << endl;
		}

		// print out each detection
		cout << detections.size() << " Camera 2 tags detected:" << endl;
		for (int i=0; i<detections.size(); i++) {
			print_detection(detections[i]);
		}

		// show the current image including any detections
		if (m_draw) {
			for (int i=0; i<detections.size(); i++) {
				// also highlight in the image
				detections[i].draw(image);
			}
			imshow(windowName1, image); // OpenCV call
		}

		// optionally send tag information to serial port (e.g. to Arduino)
		if (m_arduino) {
			if (detections.size() > 0) {
				// only the first detected tag is sent out for now
				Eigen::Vector3d translation;
				Eigen::Matrix3d rotation;
				detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
						translation, rotation);
				m_serial.print(detections[0].id);
				m_serial.print(",");
				// m_serial.print(translation(0));
				m_serial.print(",");
				//m_serial.print(translation(1));
				m_serial.print(",");
				// m_serial.print(translation(2));
				m_serial.print("\n");
			} else {
				// no tag detected: tag ID = -1
				m_serial.print("-1,0.0,0.0,0.0\n");
			}
		}
	}
	 */

	// Load and process a single image
	void loadImages() {
		cv::Mat image;
		cv::Mat image_gray;

		for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
			image = cv::imread(*it); // load image with opencv
			processImage(image, image_gray);
			while (cv::waitKey(100) == -1) {}
		}
	}

	// Video or image processing?
	bool isVideo() {
		return m_imgNames.empty();
	}

	// The processing loop where images are retrieved, tags detected,
	// and information about detections generated
	int rand = 1900;
	void loop() {

		procPort = true;

		cv::Mat image;
		cv::Mat image_gray;

		cv::Mat image1;
		cv::Mat image_gray1;

		int frame = 0;
		double last_t = tic();
		double time_last_located = tic();
		while (true) {


			//processing the port camera
			if(procPort){
				// capture frame
				m_cap >> image;

				//TODO: clear vector cache

				cout << "camera 0" << endl;
				cacheCameraIndex = 0;
				bool result = processImage(image, image_gray);
				/*				if(result){
					cout << "communicating data..." << endl;
					time_last_located = tic();
				}*/

				cout << "camera 1" << endl;
				cacheCameraIndex = 1;
				m_cap1 >> image;
				result = processImage(image, image_gray);
				/*				if(result){
					cout << "communicating data1..." << endl;
					time_last_located = tic();
				}*/

				//TODO: calculate standardized vector using vector cache
				cacheCameraIndex = -1;

				cout << "calc stand" << endl;
				cout << "\tTag 1:" << c0CacheVector0(0)
					<< " " << c0CacheVector0(1)
					<< " " << c0CacheVector0(2);
					cout << endl;
				cout << "\tTag 3:" << c0CacheVector1(0)
					<< " " << c0CacheVector1(1)
					<< " " << c0CacheVector1(2);
					cout << endl;

				Eigen::Vector3d helper = c0CacheVector1 - c0CacheVector0;
				cout << "\tHelper:" << endl;
				cout << "\t\t distance=" << helper.norm()
					<< "m x=" << helper(0)
					<< " y=" << helper(1)
					<< " z=" << helper(2);
					cout << endl;

				helper = 1.502 * helper;

				cout << "\tHelper Scaled:" << endl;
				cout << "\t\t distance=" << helper.norm()
					<< "m x=" << helper(0)
					<< " y=" << helper(1)
					<< " z=" << helper(2);
					cout << endl;

				Eigen::Vector3d standardizedVector = c0CacheVector0 + helper;

				cout << "\tStandardized Vector:" << endl;
				cout << "\t\t distance=" << standardizedVector.norm()
					<< "m x=" << standardizedVector(0)
					<< " y=" << standardizedVector(1)
					<< " z=" << standardizedVector(2);
					cout << endl;

			}

			//TODO: discuss whethor or not this is still needed in implementation
			/*			//checking time with last seen apriltag
			if(tic() - time_last_located >= 5.0){
				//cout << "Time Alloted without detection, turning on all cameras" << endl;
				if(!procPort){
					procPort = true;
				}
			}

			//Check to make sure at-least one camera is set to process
			if(!procPort){
				cout << "No Camera's Running, turning all on" << endl;
				procPort = true;
			}*/


			//sleep(2);
			// print out the frame rate at which image frames are being processed
			/*			frame++;
			if (frame % 10 == 0) {
				double t = tic();
				cout << "  " << 10./(t-last_t) << " fps" << endl;
				last_t = t;
			}*/

			//processing other frames without worrying about the rate above
			// m_cap1 >> image1;
			// processImage2(image1, image_gray1);

			/*m_cap2 >> image;
      	  processImage(image, image_gray);*/


			AMQPExchange * ex = amqp.createExchange("amq.topic");
			ex->Declare("amq.topic", "topic", AMQP_DURABLE);
			char msg [50];
			sprintf(msg,"%d",rand);
			ex->Publish(msg, sizeof(msg), "loc.post");
			rand++;
			//sleep(3);

			// exit if any key is pressed
			if (cv::waitKey(10) >= 0) break;

		}
	}

}; // Demo

// here is were everything begins
int main(int argc, char* argv[]) {



	Demo demo;

	/* Camera Calibration */


	// process command line options

	demo.setup();

	if (demo.isVideo()) {
		cout << "Processing video" << endl;

		// setup image source, window for drawing, serial port...
		demo.setupVideo();

		// the actual processing loop where tags are detected and visualized
		demo.loop();

	} else {
		cout << "Processing image" << endl;

		// process single image
		demo.loadImages();

	}

	return 0;
}
