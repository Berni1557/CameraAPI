#include "KinectV2.h"
#include <thread>

KinectV2::KinectV2()
{

}

KinectV2::~KinectV2()
{
}


int KinectV2::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{
	
	switch (camInterface) {
	case IKinectV2SDK:
	{
		// init RGB camera
		this->rgbcam = new KinectV2RGB();
		this->rgbcam->cameraname = cameraname + "_rgb";

		// init Depth camera
		this->depthcam = new KinectV2Depth();
		this->depthcam->cameraname = cameraname + "_depth";

		// init RGB camera interface
		this->rgbcam->icamera = new KinectV2SDKRGB(this->rgbcam);
		this->rgbcam->icamera->init();

		// init depth camera interface
		this->depthcam->icamera = new KinectV2SDKDepth(this->depthcam);
		this->depthcam->icamera->init();

		// init RGBD camera interface
		this->icamera = new KinectV2SDK(this, (KinectV2SDKRGB*)this->rgbcam->icamera, (KinectV2SDKDepth*)this->depthcam->icamera);
		this->icamera->init();

		return 0;
	}
	default:
	{
		return -1;
	}
	}
	
	return -1;
}

int KinectV2::aquisition()
{
	return this->icamera->aquistion();
}

int KinectV2::close()
{
	return this->rgbcam->icamera->close();
	return this->depthcam->icamera->close();
}

int KinectV2::onRGBframe(cv::Mat & image)
{
	this->rgbcam->imageRGB = ImageRGB(image);
	imshow("Mwindow", image);
	return 0;
}

int KinectV2::onDepthframe(cv::Mat & image)
{
	this->depthcam->imageDepth = ImageDepth(image);
	imshow("Mwindow", image);
	return 0;
}

int KinectV2::onframe(cv::Mat & image)
{
	return 0;
}

KinectV2Depth::KinectV2Depth()
{
}

KinectV2Depth::~KinectV2Depth()
{
}

int KinectV2Depth::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{
	/*
	this->cameraname = cameraname + "_depth";

	switch (camInterface) {
	case IIntelRealsenseF200SDK:
	{
		//this->RGBCamera::icamera = new IntelRealsenseF200SDK(this);
		//this->DepthCamera::icamera = new IntelRealsenseF200SDK(this);
		this->icamera = new IntelRealsenseF200SDKDepth(this);
		this->icamera->init();
		return 0;
	}
	default:
	{
		return -1;
	}
	}
	*/
	return -1;
}

int KinectV2Depth::aquisition()
{
	return this->icamera->aquistion();
}

int KinectV2Depth::close()
{
	return this->icamera->close();
}

int KinectV2Depth::onframe(cv::Mat & image)
{
	this->imageDepth = ImageDepth(image);
	cv::namedWindow("Depth image", cv::WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Depth image", image);                   // Show our image inside it.
	return 0;
}

KinectV2RGB::KinectV2RGB()
{
}

KinectV2RGB::~KinectV2RGB()
{
}

int KinectV2RGB::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{
	
	this->cameraname = cameraname + "_rgb";

	switch (camInterface) {
	case IKinectV2SDK:
	{
		//this->RGBCamera::icamera = new IntelRealsenseF200SDK(this);
		//this->DepthCamera::icamera = new IntelRealsenseF200SDK(this);
		this->icamera = new KinectV2SDKRGB(this);
		this->icamera->init();
		return 0;
	}
	default:
	{
		return -1;
	}
	}
	
	return -1;
}

int KinectV2RGB::aquisition()
{
	return this->icamera->aquistion();
}

int KinectV2RGB::close()
{
	return this->icamera->close();
}

int KinectV2RGB::onframe(cv::Mat & image)
{
	this->updateCamListener(image);
	//this->imageRGB = ImageRGB(image);
	//cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
	//cv::imshow("Display window", image);                   // Show our image inside it
	return 0;
}

