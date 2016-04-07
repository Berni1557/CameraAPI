#include "CameraFW.h"

CameraFW::CameraFW()
{
}

CameraFW::~CameraFW()
{
}

int CameraFW::addCameraAPI(CameraAPI* cameraAPI)
{
	this->cameralist.push_back(cameraAPI);
	return 0;
}

int CameraFW::getCameraAPI(int cameranumber, CameraAPI* cameraAPI)
{
	if(cameranumber<cameralist.size()){
		cameraAPI=this->cameralist[cameranumber];
		return 0;
	}
	else {
		return -1;
	}
	
}
