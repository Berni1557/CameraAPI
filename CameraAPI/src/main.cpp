/*******************************************************************************

Intel Realsene SDK
The program starts Color and Depth Stream using Intel Realsense SDK
and converting its frame from PXCImage to Mat variable.
Easy for Image processing in Intel Realsense SDK Camera.

*******************************************************************************/


#include "main.h"
#include "IntelRealsenseF200.h"
#include "KinectV2.h"
#include "A4CamListener.h"

#include <iostream>
#include <sstream>

#include <Windows.h>
#include <Kinect.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <QtWidgets>
#include <QtGui/QImage>
#include <QDebug>
#include <QLabel>

#include <QApplication>
#include <QCommandLineParser>
#include <QtGui>

#include <thread>
#include <iostream> 
#include <time.h> 
//#include "imageviewer.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
	
	KinectV2 Cam;
	Cam.init("KinectV2", 1, IKinectV2SDK);

	// add Listener
	A4CamListener listener;
	Cam.rgbcam->attachCamListener(&listener);

	Cam.aquisition();
	
	Cam.close();
	return 0;
	



	/*
	// threshold test
	Mat im_gray = imread("C:\\Users\\berni\\Desktop\\Testdata_20160401\\image010.tif", CV_LOAD_IMAGE_GRAYSCALE);
	Mat img_bw;
	Mat img_bwg;
	Mat img_bwc= Mat::zeros(im_gray.rows, im_gray.cols, CV_8UC3);;
	
	GaussianBlur(im_gray, im_gray, Size(3, 3), 0, 0);
	cv::equalizeHist(im_gray, im_gray);
	cv::adaptiveThreshold(im_gray, img_bw, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 51, -20);

	//threshold(im_gray, img_bwg, 100, 255, THRESH_BINARY);

	namedWindow("Gray", WINDOW_AUTOSIZE);
	imshow("Gray", im_gray);

	namedWindow("BW", WINDOW_AUTOSIZE);
	imshow("BW", img_bw);

	vector<vector<Point> > contours;
	vector<vector<Point> > contoursfa;
	vector<vector<Point> > contoursfb;
	vector<vector<Point> > contoursfr;
	vector<Vec4i> hierarchy;

	findContours(img_bw, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point(0, 0));

	// filter by area
	for (int i = 0; i< contours.size(); i++)
	{
		//  Find the area of contour
		double a = contourArea(contours[i], false);
		if (a>150 && a<10000) {
			contoursfa.push_back(contours[i]);
		}
	}
	// filter by bounding box
	Rect bounding_rect;
	for (int i = 0; i< contoursfa.size(); i++)
	{
		//  Find the area of contour
		bounding_rect = boundingRect(contoursfa[i]);
		if (bounding_rect.width>10 && bounding_rect.width<60 && bounding_rect.height>5 && bounding_rect.height<40) {
			contoursfb.push_back(contoursfa[i]);
		}
	}
	*/


	/*
	Moments moms;
	for (int i = 0; i < contoursfb.size(); i++){
		moms = moments(Mat(contoursfb[i]));
		double denominator = sqrt(pow(2 * moms.mu11, 2) + pow(moms.mu20 - moms.mu02, 2));
		const double eps = 1e-2;
		double ratio;
		if (denominator > eps)
		{
			double cosmin = (moms.mu20 - moms.mu02) / denominator;
			double sinmin = 2 * moms.mu11 / denominator;
			double cosmax = -cosmin;
			double sinmax = -sinmin;

			double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
			double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
			ratio = imin / imax;
		}
		else
		{
			ratio = 1;
		}
		if(ratio>0.02){
			contoursfr.push_back(contoursfb[i]);
		}
	}
	*/

	/*
	// circularity
	const double PI = 3.141592653589793238463;
	double circ;
	for (int i = 0; i < contoursfb.size(); i++) {
		double a = contourArea(contoursfb[i], false);
		double perimeter = arcLength(Mat(contoursfb[i]), true);
		circ = (4 * PI * a) / (perimeter*perimeter);
		if (circ>0.6) {
			contoursfr.push_back(contoursfb[i]);
		}
	}
	
	Scalar color(255, 255, 255);
	drawContours(img_bwc, contoursfb, -1, color);

	namedWindow("BWc", WINDOW_AUTOSIZE);
	imshow("BWc", img_bwc);

	waitKey(0);
	return 0;
	*/


	/*
	IntelRealsenseF200 Cam;
	Cam.init("IntelRealsenseF200", 1, IIntelRealsenseF200SDK);

	// add Listener
	//CamListener listener;
	//Cam.attachCamListener(&listener);

	Cam.aquisition();
	Cam.close();
	*/


	return 0;
}


