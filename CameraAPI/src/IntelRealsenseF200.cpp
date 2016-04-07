#include "IntelRealsenseF200.h"

IntelRealsenseF200::IntelRealsenseF200()
{
	
}

IntelRealsenseF200::~IntelRealsenseF200()
{
}

int IntelRealsenseF200::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{

	switch (camInterface) {
		case IIntelRealsenseF200SDK:
		{
			// init RGB camera
			this->rgbcam = new IntelRealsenseF200RGB();
			this->rgbcam->cameraname = cameraname + "_rgb";

			// init Depth camera
			this->depthcam = new IntelRealsenseF200Depth();
			this->depthcam->cameraname = cameraname + "_depth";

			// init RGB camera interface
			this->rgbcam->icamera= new IntelRealsenseF200SDKRGB(this->rgbcam);
			this->rgbcam->icamera->init();

			// init depth camera interface
			this->depthcam->icamera = new IntelRealsenseF200SDKDepth(this->depthcam);
			this->depthcam->icamera->init();

			// init RGBD camera interface
			this->icamera = new IntelRealsenseF200SDK(this, (IntelRealsenseF200SDKRGB*)this->rgbcam->icamera, (IntelRealsenseF200SDKDepth*)this->depthcam->icamera);
			this->icamera->init();

			return 0;
		}
		default:
		{
			return -1;
		}
	}
}

int IntelRealsenseF200::aquisition()
{
	return this->icamera->aquistion();
}

int IntelRealsenseF200::close()
{
	return this->rgbcam->icamera->close();
	return this->depthcam->icamera->close();
}

int IntelRealsenseF200::onRGBframe(cv::Mat & image)
{
	this->rgbcam->imageRGB = ImageRGB(image);
	imshow("Mwindow", image);
	return 0;
}

int IntelRealsenseF200::onDepthframe(cv::Mat & image)
{
	this->depthcam->imageDepth = ImageDepth(image);
	imshow("Mwindow", image);
	return 0;
}

int IntelRealsenseF200::onframe(cv::Mat & image)
{
	return 0;
}

IntelRealsenseF200Depth::IntelRealsenseF200Depth()
{
}

IntelRealsenseF200Depth::~IntelRealsenseF200Depth()
{
}

int IntelRealsenseF200Depth::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{
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
}

int IntelRealsenseF200Depth::aquisition()
{
	return this->icamera->aquistion();
}

int IntelRealsenseF200Depth::close()
{
	return this->icamera->close();
}

int IntelRealsenseF200Depth::onframe(cv::Mat & image)
{
	this->imageDepth = ImageDepth(image);
	cv::namedWindow("Depth image", cv::WINDOW_AUTOSIZE);// Create a window for display.
	imshow("Depth image", image);                   // Show our image inside it.
	return 0;
}

IntelRealsenseF200RGB::IntelRealsenseF200RGB()
{
}

IntelRealsenseF200RGB::~IntelRealsenseF200RGB()
{
}

int IntelRealsenseF200RGB::init(std::string cameraname, int cameraNumber, CameraInterfaceEnum camInterface)
{
	this->cameraname = cameraname + "_rgb";

	switch (camInterface) {
		case IIntelRealsenseF200SDK:
		{
			//this->RGBCamera::icamera = new IntelRealsenseF200SDK(this);
			//this->DepthCamera::icamera = new IntelRealsenseF200SDK(this);
			this->icamera = new IntelRealsenseF200SDKRGB(this);
			this->icamera->init();
			return 0;
		}
		default:
		{
			return -1;
		}
	}
}

int IntelRealsenseF200RGB::aquisition()
{
	return this->icamera->aquistion();
}

int IntelRealsenseF200RGB::close()
{
	return this->icamera->close();
}

int IntelRealsenseF200RGB::onframe(cv::Mat & image)
{
	this->updateCamListener(image);
	this->imageRGB = ImageRGB(image);
	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
	cv::imshow("Display window", image);                   // Show our image inside it
	return 0;
}

