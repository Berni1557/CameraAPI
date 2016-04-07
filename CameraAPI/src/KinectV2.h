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

#ifndef KinectV2_H
#define KinectV2_H

#include "KinectV2SDK.h"
//#include "RGBDCamera.h"
#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/// class IntelRealsenseF200 - 
class KinectV2 : public RGBDCamera {

public:
	KinectV2();
	~KinectV2();
	int init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface);
	int aquisition();
	int close();
	int onRGBframe(cv::Mat& imageRGB);
	int onDepthframe(cv::Mat& imageDepth);
	int onframe(cv::Mat& image);
};

class KinectV2RGB : public RGBCamera {

public:
	KinectV2RGB();
	~KinectV2RGB();
	int init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface);
	int aquisition();
	int close();
	int onframe(cv::Mat& image);
};

class KinectV2Depth : public DepthCamera {

public:
	KinectV2Depth();
	~KinectV2Depth();
	int init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface);
	int aquisition();
	int close();
	int onframe(cv::Mat& image);
};

#endif // KinectV2_H

