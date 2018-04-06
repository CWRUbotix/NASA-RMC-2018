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

#include "amqpcpp/AMQPcpp.h"
#include <amqpcpp/AMQPcpp.h>

#include "messages.pb.h"

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

const char* windowGreenCam = "Green Cam";
const char* windowYellowCam = "Yellow Cam";

/*
 *
 * Standardization
 *
 */
struct tagCache{

	int tagID;
	string cameraID;
	Eigen::Vector3d capturedVector;

};

//database for captured tags
std::vector<tagCache> capturedTagCaches;

//distance from tag with named ID to the standardization point (int meters)
const float distToStandard01 = 0.545f;
const float distToStandard03 = 0.3245f;
const float distToStandard05 = 0.108f;
const float distToStandard09 = -0.11f;
const float distToStandard10 = -0.326f;
const float distToStandard11 = -0.5425f;


// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
	struct timeval t;
	gettimeofday(&t, NULL);
	return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}


#include <cmath>
#include <math.h>
#include <stdio.h>

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

//Camera Structure
struct camera {
	double c_fx;	//camera focal length for green cam
	double c_fy;
	double c_px;	//camera principal length for green cam
	double c_py;
	int c_id;
};

camera greenCam = {644.12, 644.12, 319.5, 239.5, 0};
camera yellowCam = {538.23, 538.23, 319.5, 239.5, 1};

class Localization {

	AprilTags::TagDetector* m_tagDetector;
	AprilTags::TagCodes m_tagCodes;

	bool m_draw; // draw image and April tag detections?
	bool m_arduino; // send tag detections to serial port?
	bool m_timing; // print timing information for each tag extraction call

	int m_width; // image size in pixels
	int m_height;
	double m_tagSize; // April tag side length in meters of square black frame

	list<string> m_imgNames;

	//camera greenCam, yellowCam;
	//camera greenCam = {644.12, 644.12, 319.5, 239.5, 0};
	//camera yellowCam = {538.23, 538.23, 319.5, 239.5, 1};
	cv::VideoCapture green_cap;
	cv::VideoCapture yellow_cap;

	int m_exposure;
	int m_gain;
	int m_brightness;

	Serial m_serial;

	bool procPort = false;
	bool procStarboard = false;
	bool procStern = false;

public:

	// default constructor
	Localization() :
		// default settings, most can be modified through command line options (see below)
		m_tagDetector(NULL),
		m_tagCodes(AprilTags::tagCodes36h11),

		m_draw(true),
		m_arduino(false),
		m_timing(false),
		m_width(640),
		m_height(480),
		m_tagSize(0.165),

		/*green_fx(644.12),
		green_fy(644.12),
		green_px(319.5),
		green_py(239.5),

		yellow_fx(538.23),
		yellow_fy(538.23),
		yellow_px(319.5),
		yellow_py(239.5),*/

		m_exposure(-1),
		m_gain(-1),
		m_brightness(-1)

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
			cv::namedWindow(windowGreenCam, 1);
			cv::namedWindow(windowYellowCam, 1);
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

				string video_str = "/dev/video0";
				video_str[10] = '0' + greenCam.c_id;
				int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

				string video_str1 = "/dev/video1";
				video_str1[10] = '1' + yellowCam.c_id;
				int device1 = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

				//string video_str2 = "/dev/video2";
				//video_str2[10] = '2' + m_deviceId;

				//int device2 = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

				if (m_exposure >= 0) {
					// not sure why, but v4l2_set_control() does not work for
					// V4L2_CID_EXPOSURE_AUTO...
					struct v4l2_control c_green;
					c_green.id = V4L2_CID_EXPOSURE_AUTO;
					c_green.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
					if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c_green) != 0) {
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

				//if statements for the other cameras
				if (m_exposure >= 0) {
					struct v4l2_control c_yellow;
					c_yellow.id = V4L2_CID_EXPOSURE_AUTO;
					c_yellow.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
					if (v4l2_ioctl(device1, VIDIOC_S_CTRL, &c_yellow) != 0) {
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


		#endif

		// find and open robot cameras
		green_cap = cv::VideoCapture(greenCam.c_id);
		if(!green_cap.isOpened()) {
			cerr << "ERROR: Can't find Green Camera" << "\n";
			exit(1);
		}

		green_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		green_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

		cout << "Green Camera successfully opened (ignore error messages above...)" << endl;
		cout << "Actual resolution: "
				<< green_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
				<< green_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

		yellow_cap = cv::VideoCapture(yellowCam.c_id);
		if(!yellow_cap.isOpened()) {
			cerr << "ERROR: Can't find Yellow Camera" << "\n";
			exit(1);
		}
		yellow_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
		yellow_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);

		cout << "Yellow Camera successfully opened (ignore error messages above...)" << endl;
		cout << "Actual resolution: "
				<< yellow_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
				<< yellow_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

	}

	//TODO: Take into account the fact that each camera has unique
	//calibration values. Maybe make a camera struct and include appropriate fields
	//in functions that utilize calibration values

	void print_detection(AprilTags::TagDetection& detection, camera camera, string name) const {
		/*cout << "  Id: " << detection.id
				<< " (Hamming: " << detection.hammingDistance << ")";
		void print_detection(AprilTags::TagDetection& detection, std::vector<tagCache>& dataRecords, int cameraID) const {
			cout << "\t\t  Id: " << detection.id
					<< " (Hamming: " << detection.hammingDistance << ")";*/


		// recovering the relative pose of a tag:

		// NOTE: for this to be accurate, it is necessary to use the
		// actual camera parameters here as well as the actual tag size
		// (m_fx, m_fy, m_px, m_py, m_tagSize)

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		detection.getRelativeTranslationRotation(m_tagSize, camera.c_fx, camera.c_fy, camera.c_px, camera.c_py,
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
		/*		cout << "  distance=" << translation.norm()
        										 << "m, x=" << translation(0)
												 << ", y=" << translation(1)
												 << ", z=" << translation(2)
												 << ", yaw(x)=" << yaw
												 << ", pitch(z)=" << pitch
												 << ", roll(y)=" << roll;
		cout   << endl;*/

		tagCache toAdd;
		toAdd.cameraID = name;
		toAdd.tagID = detection.id;
		toAdd.capturedVector = translation;

		capturedTagCaches.push_back(toAdd);
	}


	bool processImage(cv::Mat& image, cv::Mat& image_gray, camera camera, string name) {

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
		//cout << "\t" << detections.size() << " Camera tags detected:" << endl;
		bool ret = false;
		//cout << "\tdetections.size: "<< detections.size() << endl;
		for (int i=0; i<detections.size(); i++){
			print_detection(detections[i], camera, name); //previously commented out
			//TODO fix this part print_detection(detections[i], capturedTags, cameraID);
			ret = true;
		}

		// show the current image including any detections
		if (m_draw) {
			for (int i=0; i<detections.size(); i++) {
				// also highlight in the image
				detections[i].draw(image);
			}
			imshow(name, image); // OpenCV call
		}

		// optionally send tag information to serial port (e.g. to Arduino)
		if (m_arduino) {
			if (detections.size() > 0) {
				// only the first detected tag is sent out for now
				Eigen::Vector3d translation;
				Eigen::Matrix3d rotation;
				detections[0].getRelativeTranslationRotation(m_tagSize, camera.c_fx, camera.c_fy, camera.c_px, camera.c_py,
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


		return ret;
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
		cv::Mat image;
		cv::Mat image_gray;

		cv::Mat yellow_image;
		cv::Mat yellow_image_gray;

		int frame = 0;
		double last_t = tic();
		double time_last_located = tic();
		while (true) {

			green_cap >> green_image;
			cout << "communicating data..." << endl;
			yellow_cap >> yellow_image;
			cout << "communicating data1..." << endl;
			processImage(green_image, green_image_gray, greenCam, windowGreenCam);
			processImage(yellow_image, yellow_image_gray, greenCam, windowYellowCam);

			//TODO: uncomment
			/*com::cwrubotix::glennifer::LocalizationPosition msg;
				float x = 10;
				float y = 15;
				msg.set_x_position(x);
				msg.set_y_position(y);
				//msg.set_timeout(456);
				int msg_size = msg.ByteSize();
				void *msg_buff = malloc(msg_size);
				msg.SerializeToArray(msg_buff, msg_size);
				AMQPExchange * ex = amqp.createExchange("amq.topic");
				ex->Declare("amq.topic", "topic", AMQP_DURABLE);
				AMQPQueue *queue = amqp.createQueue("localization.data");
				queue->Declare();
				queue->Bind("amq.topic", "localization.data");
				ex->Publish((char*)msg_buff, msg_size, "localization.data");*/

			//sleep(100);

			//AMQPExchange * ax = amqp.createExchange("amq.topic");
			//ax->Declare("amq.topic", "topic", AMQP_DURABLE);

			//AMQPQueue * qu2 = amqp.createQueue("localization.data");
			//qu2->Declare();
			//qu2->Bind( "amq.topic", "localization.data");
			//ax->Publish((char*)msg_buff, msg_size, "localization.data");

			cout << "num of cached instances=" << capturedTagCaches.size() << endl;

			std::vector<Eigen::Vector3d> foundStandards;

			int length = capturedTagCaches.size();
			for(int i = 0; i < length; i++){
				for(int j = 0; j < length; j++){
					if(j != i){

						if(capturedTagCaches.at(i).cameraID == capturedTagCaches.at(j).cameraID){

							cout << "\tchecking camera " << capturedTagCaches.at(i).cameraID
									<< " which caught tag " << capturedTagCaches.at(i).tagID
									<< " and tag " << capturedTagCaches.at(j).tagID << endl;

							//create standardized vector
							float a = 0.0f;
							float b = 0.0f;
							switch(capturedTagCaches.at(i).tagID){
							case 1:
								a = distToStandard01;
								break;
							case 3:
								a = distToStandard03;
								break;
							case 5:
								a = distToStandard05;
								break;
							case 9:
								a = distToStandard09;
								break;
							case 10:
								a = distToStandard10;
								break;
							case 7:
								a = distToStandard11;
								break;
							default:
								a = 0.0f;
							}

							switch(capturedTagCaches.at(j).tagID){
							case 1:
								b = distToStandard01;
								break;
							case 3:
								b = distToStandard03;
								break;
							case 5:
								b = distToStandard05;
								break;
							case 9:
								b = distToStandard09;
								break;
							case 10:
								b = distToStandard10;
								break;
							case 7:
								b = distToStandard11;
								break;
							default:
								b = 0.0f;
							}

							cout << "\t\ta = " << a << " b = " << b << endl;

							Eigen::Vector3d stdFromCamera;

							if((a > 0 && b > 0) && a > b){//++

								cout << "\t\tfound as a ++" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(i).capturedVector - capturedTagCaches.at(j).capturedVector;
								float scaler = (a) / (a - b);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(i).capturedVector + helper2;


							}else if((a > 0 && b > 0) && b > a){//++ fliped

								cout << "\t\tfound as a ++ flip" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(j).capturedVector - capturedTagCaches.at(i).capturedVector;
								float scaler = (b) / (b - a);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(j).capturedVector + helper2;

							}else if((a < 0 && b < 0) && a < b){//--

								cout << "\t\tfound as a --" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(i).capturedVector - capturedTagCaches.at(j).capturedVector;
								float scaler = (a) / (a - b);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(i).capturedVector + helper2;

							}else if((a < 0 && b < 0) && a > b){//-- fliped

								cout << "\t\tfound as a -- flip" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(j).capturedVector - capturedTagCaches.at(i).capturedVector;
								float scaler = (b) / (b - a);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(j).capturedVector + helper2;

							}else if(a > 0 && b < 0){//+-

								cout << "\t\tfound as a +-" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(i).capturedVector - capturedTagCaches.at(j).capturedVector;
								float scaler = (a) / (a - b);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(i).capturedVector + helper2;

							}else if(a < 0 && b > 0){//-+

								cout << "\t\tfound as a -+" << endl;
								Eigen::Vector3d helper1 = capturedTagCaches.at(i).capturedVector - capturedTagCaches.at(j).capturedVector;
								float scaler = (a) / (a - b);
								Eigen::Vector3d helper2 = scaler * helper1;
								stdFromCamera = capturedTagCaches.at(i).capturedVector + helper2;

							}else{
								cout << "\t\tERROR, undefined combination for 2 tags a = " << a << " b = " << b << endl;
							}

							cout << "\t\tStdVector:" << endl;

							cout << "\t\t\tdistance=" << stdFromCamera.norm()
																				<< "m x=" << stdFromCamera(0)
																				<< " y=" << stdFromCamera(1)
																				<< " z=" << stdFromCamera(2);
							cout << endl;

							//add amqp
							AMQPExchange * ex = amqp.createExchange("amq.topic");
							ex->Declare("amq.topic", "topic", AMQP_DURABLE);
							com::cwrubotix::glennifer::LocalizationPosition msg;
							//float x = 10;
						    //float y = 15;
							msg.set_x_position(stdFromCamera(0));
							msg.set_y_position(stdFromCamera(1));
							int msg_size = msg.ByteSize();
						    void *msg_buff = malloc(msg_size);
							msg.SerializeToArray(msg_buff, msg_size);
							ex->Publish((char*)msg_buff, msg_size, "loc.post");

							foundStandards.push_back(stdFromCamera);

							/*double result;
							result = 90 - (acos (stdFromCamera(1)/stdFromCamera.norm()) * 180 / PI);

							cout << "Angle is: " << result << "\n" << endl;*/

						}
					}
				}
			}


			float norm,x,y,z;

			for(int i = 0; i < foundStandards.size(); i++){
				norm += foundStandards.at(i).norm();
				x += foundStandards.at(i)(0);
				y += foundStandards.at(i)(1);
				z += foundStandards.at(i)(2);
			}

			norm = norm / foundStandards.size();
			x = x / foundStandards.size();
			y = y / foundStandards.size();
			z = z / foundStandards.size();

			cout << "\tOVERALL VECTOR:\n"
					<< "\t\tdist=" << norm
					<< " x=" << x
					<< " y=" << y
					<< " z=" << z << endl;

			double result;
			result = (acos (x/norm) * 180 / PI);

			cout << "Angle is: " << result << "\n" << endl;

			capturedTagCaches.clear();

			// exit if any key is pressed
			if (cv::waitKey(10) >= 0) break;
		    sleep(1);

		}
	}

};

// here is were everything begins
int main(int argc, char* argv[]) {

	//init_Queue();

	Localization localization;

	/* Camera Calibration */


	// process command line options

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
		//localization.loadImages();

	}

	return 0;
}
