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

#ifndef CAMERAAPI_H
#define CAMERAAPI_H


#include <iostream>
#include <memory>
#include <stdio.h>


#include "CameraInterfaceEnum.h"
#include "ICamera.h"
#include "CalibrationModel.h"
#include <opencv2/core/core.hpp>
#include "CircBuffer.h"
#include "QtCamListener.h"

#include <QtWidgets>
#include <QtGui/QImage>
#include <QDebug>

class ICamera;
// interface
/// class CameraAPI - 
class CameraAPI {


  // Attributes
public:
  std::string cameraname;
  ICamera* icamera;
protected:
	
	//CircBuffer<cv::Mat> circBuffer;
	CalibrationModel* calibrationModel;
   int cameraNumber;
  // Operations
public:
  CameraAPI();
  ~CameraAPI();
  virtual int init (std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface) = 0;
  virtual int aquisition () = 0;
  virtual int close () = 0;
  virtual int onframe (cv::Mat& image) = 0;
  int setCalibrationModel (CalibrationModel& calibrationModel);
  int setCameraInterface (ICamera& cameraInterface);
  CalibrationModel* getCalibrationModel ();

  int attachCamListener(ICamListener* listener);
  int detachCamListener(ICamListener* listener);
  int updateCamListener(cv::Mat& image);

  //int createCameraInterface (ICamera* cameraInterface, CameraInterfaceEnum cameraInterfaceEnum);
private:
	std::vector<ICamListener*> CamListenerList;
};

#endif // CAMERAAPI_H
