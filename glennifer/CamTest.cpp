/*
 * CamTest.cpp
 *
 *
 */
#include <highgui.h>
#include <cv.hpp>
#include <cxcore.hpp>
#include <cxcore.h>
#include <cvaux.hpp>
#include <cvaux.h>
#include <cv.h>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;
typedef unsigned char byte;

VideoCapture cam0(0);
VideoCapture cam1(1);

Mat bytesToMat(byte * bytes,int width,int height) {
    Mat image = Mat(height,width,CV_8UC3,bytes).clone(); // make a copy
    return image;
}

byte * matToBytes(Mat image) {
   int size = image.total() * image.elemSize();
   byte * framesend = new byte[size];
   memcpy(framesend,image.data,size * sizeof(byte));
   return framesend;
}

//get integrated camera frame
cv::Mat goGetFrame0(){
	Mat temp0;
	while(!cam0.isOpened()) {
		cout << "Camera Zero is not Open" << endl;
	}
	cam0 >> temp0;
	return temp0;

}

//get carl zeiss tessar hd logitech camera
cv::Mat goGetFrame1(){
	Mat temp;
	while(!cam1.isOpened()) {
		cout << "Camera One is not Open" << endl;
	}
	cam1 >> temp;
	return temp;

}

int main() {
	int one = 0;

	if(!cam0.isOpened()){
			cout << "camera1 cannot be opened";
		}

	if(!cam1.isOpened()){
			cout << "camera1 cannot be opened";
		}
	while(true) {
		//Camera 0
		string onestring0 = "" + (char)one;
		string onepath0 = "/home/phoenix/Pictures/TestRun0/frame1-" + onestring0;
		namedWindow("cam0", WINDOW_AUTOSIZE);
		//Mat framef0;
		Mat frame0;
		frame0 = goGetFrame0().clone();
		//cvtColor(framef0,frame0, CV_BGR2RGB); //transforms from bgr2rgb
		string path0 = onepath0 + ".jpg";
		imwrite(path0, frame0);
		imshow("cam0", frame0);

		//Camera 1
		string onestring = "" + (char)one;
		string onepath = "/home/phoenix/Pictures/TestRun1/frame1-" + onestring;
		namedWindow("cam1", WINDOW_AUTOSIZE);
		//Mat framef1;
		Mat frame1;
		frame1 = goGetFrame1().clone();
		//cvtColor(framef1, frame1, CV_BGR2RGB); //transforms from bgr2rgb
		string path = onepath + ".jpg";
		//matToBytes(frame1);
		imwrite(path, frame1);
		imshow("cam1", frame1);

		one++;

		int c = cvWaitKey(40);
		if(27 == char(c))
			break;
	}

	return 0;
}



