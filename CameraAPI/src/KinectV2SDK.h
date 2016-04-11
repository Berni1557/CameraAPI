//  Copyright (c) 2016, Bernhard Föllmer	
//  All rights reserved.				
//  					
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//  3. All advertising materials mentioning features or use of this software
//     must display the following acknowledgement:
//     This product includes software developed by the <organization>.
//  4. Neither the name of the <organization> nor the
//     names of its contributors may be used to endorse or promote products
//     derived from this software without specific prior written permission.
//  		
//  THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef KinectV2SDK_H
#define KinectV2SDK_H

#include "ICamera.h"
#include <Kinect.h>
/*
#include "pxcsensemanager.h"
#include "util_render.h"  //SDK provided utility class used for rendering (packaged in libpxcutils.lib)
#include "pxcsession.h"
#include "pxccapture.h"
#include "util_render.h"
#include "util_cmdline.h"
#include "pxcprojection.h"
#include "pxcmetadata.h"
#include "util_cmdline.h"
*/

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgproc/imgproc_c.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"


#include <iostream>
#include <stdio.h>

#include <CircBuffer.h>
#include <IHumanBody.h>
#include <CameraAPI.h>
#include "RGBDCamera.h"

//class IntelRealsenseF200SDKRGB;
//class IntelRealsenseF200SDKDepth;




/// class IntelRealsenseF200SDKRGB - 
class KinectV2SDKRGB : public ICamera {
	// Associations

	// Attributes
public:

	//UtilRender color_render = UtilRender(L"Color Stream");;
	IplImage * image = 0;
	CameraAPI * cameraapi;
	
private:

	//PXCSenseManager* psm;
	unsigned char * rgb_data;
	IColorFrameReader* colorFrameReader = nullptr; // color reader
	IKinectSensor* kinectSensor;

	// Operations
public:
	KinectV2SDKRGB();
	KinectV2SDKRGB(CameraAPI* cameraapi);
	~KinectV2SDKRGB();
	IHumanBody* humanBody;
	int init();
	int aquistion();
	int close();
	int onframe(cv::Mat& image);
	int createProperties();
	int setProperties();
	int setProperty(std::string name, boost::any value);
};



/// class IntelRealsenseF200SDKDepth - 
class KinectV2SDKDepth : public ICamera {
	// Associations

	// Attributes
public:
	
	//UtilRender depth_render = UtilRender(L"Depth Stream");
	IplImage * depth = 0;
	CameraAPI * cameraapi;

private:
	//PXCSenseManager* psm;
	short * depth_data;
	IDepthFrameReader* depthFrameReader = nullptr; // color reader
	IKinectSensor* kinectSensor;

	// Operations
public:
	KinectV2SDKDepth();
	KinectV2SDKDepth(CameraAPI* cameraapi);
	~KinectV2SDKDepth();
	IHumanBody* humanBody;
	int init();
	int aquistion();
	int close();
	int onframe(cv::Mat& image);
	int createProperties();
	int setProperties();
	int setProperty(std::string name, boost::any value);
	int getSDKCalibrationModel(IntrinsicParameter& intrinsic);
};

/// class IntelRealsenseF200SDK - 
class KinectV2SDK : public ICamera {
	// Associations

	// Attributes
public:

	/*
	UtilRender color_render = UtilRender(L"Color Stream");;
	UtilRender depth_render = UtilRender(L"Depth Stream");
	*/
	IplImage * image = 0;
	IplImage * depth = 0;
	CameraAPI * cameraapi;
	KinectV2SDKRGB * icameraRGB;
	KinectV2SDKDepth * icameraDepth;
	int colorFrameReaderToMat(cv::Mat& rgb);

private:
	//PXCSenseManager* psm;
	unsigned char * rgb_data;
	short * depth_data;
	IColorFrameReader* colorFrameReader; // color reader
	IKinectSensor* kinectSensor;

	// Operations
public:
	KinectV2SDK();
	KinectV2SDK(CameraAPI* cameraapi, KinectV2SDKRGB* icameraRGB, KinectV2SDKDepth* icameraDepth);
	~KinectV2SDK();
	IHumanBody* humanBody;
	int init();
	int aquistion();
	int close();
	int createProperties();
	int setProperties();
	int setProperty(std::string name, boost::any value);
	int getCalibIntelRealsenseF200();
	int drawSkeleton(cv::Mat&);
};


#endif // KinectV2SDK_H

