#include "CameraAPI.h"

CameraAPI::CameraAPI()
{
}

CameraAPI::~CameraAPI()
{
}

/*
int CameraAPI::init(std::string Cameraname, int CameraNumber, CameraInterfaceEnum camInterface)
{
	this->cameraname = Cameraname;
	this->cameraNumber = CameraNumber;
	return 0;
}

int CameraAPI::aquisition()
{
	return this->icamera->aquistion();
}

int CameraAPI::close()
{
	return this->icamera->close();
}

int CameraAPI::onframe(cv::Mat & image)
{
	return this->icamera->onframe(image);
}
*/
int CameraAPI::setCalibrationModel(CalibrationModel& calibrationModel)
{
	this->calibrationModel = &calibrationModel;
	return 0;
}

int CameraAPI::setCameraInterface(ICamera & cameraInterface)
{
	this->icamera = &cameraInterface;
	return 0;
}

CalibrationModel * CameraAPI::getCalibrationModel()
{
	return this->calibrationModel;
}


int CameraAPI::attachCamListener(ICamListener* listener)
{
	CamListenerList.push_back(listener);
	return 0;
}

int CameraAPI::detachCamListener(ICamListener * listener)
{
	CamListenerList.erase(std::remove(CamListenerList.begin(), CamListenerList.end(), listener), CamListenerList.end());
	return 0;
}

int CameraAPI::updateCamListener(cv::Mat & image)
{
	for (std::vector<ICamListener*>::const_iterator iter = CamListenerList.begin(); iter != CamListenerList.end(); ++iter)
	{
		if (*iter != 0)
		{
			(*iter)->update(image);
		}
	}
	return 0;
}



