#include "ICamera.h"


ICamera::ICamera()
{
}

ICamera::ICamera(CameraAPI* cameraapi)
{
	this->frameNumber = 0;
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
}

ICamera::~ICamera()
{
}

int ICamera::getFrameNumber()
{
	return frameNumber;
}
