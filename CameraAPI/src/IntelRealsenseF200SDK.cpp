#include "IntelRealsenseF200SDK.h"



IntelRealsenseF200SDK::IntelRealsenseF200SDK()
{
}

IntelRealsenseF200SDK::IntelRealsenseF200SDK(CameraAPI* cameraapi, IntelRealsenseF200SDKRGB* icameraRGB, IntelRealsenseF200SDKDepth* icameraDepth)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
	this->icameraRGB = icameraRGB;
	this->icameraDepth = icameraDepth;
}

IntelRealsenseF200SDK::~IntelRealsenseF200SDK()
{
}

int IntelRealsenseF200SDK::init()
{
	//this->getCalibIntelRealsenseF200();
	
	std::cout << "Intel Realsense SDK Hacking using Opencv" << std::endl;
	std::cout << "Intel Realsense Camera SDK Frame Capture in opencv Mat Variable       -- by Deepak" << std::endl;
	std::cout << "Compiled with OpenCV version " << CV_VERSION << std::endl;

	psm = 0;
	psm = PXCSenseManager::CreateInstance();

	if (!psm) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		//return 1;
	}

	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480); //depth resolution
	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480); //depth resolution
	psm->Init();

	// create Properties
	this->createProperties();

	// set Properties
	this->setProperties();

	///////////// OPENCV
	image = 0;
	CvSize gab_size;
	gab_size.height = 480;
	gab_size.width = 640;
	image = cvCreateImage(gab_size, 8, 3);

	depth = 0;
	CvSize gab_size_depth;
	gab_size_depth.height = 240;
	gab_size_depth.width = 320;
	depth = cvCreateImage(gab_size, 8, 1);

	PXCImage::ImageInfo rgb_info;
	PXCImage::ImageInfo depth_info;

	//cv::namedWindow("Mwindow", cv::WINDOW_AUTOSIZE);
	return 0;
}


int IntelRealsenseF200SDK::aquistion()
{
	PXCImage::ImageData data;
	PXCImage::ImageData data_depth;

	//CircBuffer<cv::Mat> cb(10);

	for (;;)
	{
		if (psm->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break;

		PXCCapture::Sample *sample = psm->QuerySample();
		PXCImage *colorIm, *depthIm;

		// retrieve the image or frame by type from the sample
		colorIm = sample->color;
		depthIm = sample->depth;

		PXCImage *color_image = colorIm;
		PXCImage *depth_image = depthIm;

		color_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_RGB24, &data);
		depth_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, &data_depth);

		// create rgb image
		cv::Mat rgb(480, 640, CV_8UC3);
		rgb_data = data.planes[0];
		for (int y = 0; y<480; y++)
		{
			for (int x = 0; x<640; x++)
			{
				for (int k = 0; k<3; k++)
				{
					rgb.at<cv::Vec3b>(y, x)[k] = rgb_data[y * 640 * 3 + x * 3 + k];
				}
			}
		}

		// create depth image
		depth_data = (short*)data_depth.planes[0];
		cv::Mat depth(480, 640, CV_8UC1);
		for (int x = 0; x < 480; x++)
		{
			for (int y = 0; y < 640; y++)
			{
				depth.at<uchar>(x, y) = depth_data[x * 640 + y];
			}
		}
		cv::equalizeHist(depth, depth);

		if (cvWaitKey(10) >= 0)
			break;

		this->icameraRGB->onframe(rgb);
		this->icameraDepth->onframe(depth);
		// release frame
		psm->ReleaseFrame();
	}
	return 0;
}

int IntelRealsenseF200SDK::close()
{
	cvReleaseImage(&image);
	cvReleaseImage(&depth);
	psm->Release();
	return 0;
}

int IntelRealsenseF200SDK::createProperties()
{

	// create properties
	std::string name = "IntelRealSenseF200";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	return 0;
}

int IntelRealsenseF200SDK::setProperties()
{
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	return 0;
}

int IntelRealsenseF200SDK::setProperty(std::string name, boost::any value)
{
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power= boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b=psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	return -1;
}

int IntelRealsenseF200SDK::getCalibIntelRealsenseF200()
{
	psm = 0;
	psm = PXCSenseManager::CreateInstance();
	PXCCaptureManager* cm = psm->QueryCaptureManager();
	PXCCapture::Device* d = cm->QueryDevice();
	PXCCalibration *calib = psm->QueryCaptureManager()->QueryDevice()->CreateProjection()->QueryInstance<PXCCalibration>();


	PXCCalibration::StreamCalibration* scalib;
	PXCCalibration::StreamTransform *transformation;
	pxcStatus st= calib->QueryStreamProjectionParameters(PXCCapture::StreamType::STREAM_TYPE_COLOR, scalib, transformation);

	PXCPointF32 p = scalib->focalLength;
	return 0;
}

IntelRealsenseF200SDKRGB::IntelRealsenseF200SDKRGB()
{
}

IntelRealsenseF200SDKRGB::IntelRealsenseF200SDKRGB(CameraAPI * cameraapi)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
}

IntelRealsenseF200SDKRGB::~IntelRealsenseF200SDKRGB()
{
}

int IntelRealsenseF200SDKRGB::init()
{
	std::cout << "Intel Realsense SDK Hacking using Opencv" << std::endl;
	std::cout << "Intel Realsense Camera SDK Frame Capture in opencv Mat Variable       -- by Deepak" << std::endl;
	std::cout << "Compiled with OpenCV version " << CV_VERSION << std::endl;

	psm = 0;
	psm = PXCSenseManager::CreateInstance();

	if (!psm) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		//return 1;
	}
	psm->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480); //depth resolution

	psm->Init();

	// create Properties
	this->createProperties();

	// set Properties
	this->setProperties();


	///////////// OPENCV
	image = 0;
	CvSize gab_size;
	gab_size.height = 480;
	gab_size.width = 640;
	image = cvCreateImage(gab_size, 8, 3);

	PXCImage::ImageInfo rgb_info;

	//cv::namedWindow("Mwindow", cv::WINDOW_AUTOSIZE);
	return 0;
}

int IntelRealsenseF200SDKRGB::aquistion()
{
	PXCImage::ImageData data;

	for (;;)
	{
		if (psm->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break;

		PXCCapture::Sample *sample = psm->QuerySample();
		PXCImage *colorIm;

		// retrieve the image or frame by type from the sample
		colorIm = sample->color;

		PXCImage *color_image = colorIm;

		color_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, PXCImage::PIXEL_FORMAT_RGB24, &data);

		// create rgb image
		cv::Mat rgb(480, 640, CV_8UC3);
		rgb_data = data.planes[0];
		for (int y = 0; y<480; y++)
		{
			for (int x = 0; x<640; x++)
			{
				for (int k = 0; k<3; k++)
				{
					rgb.at<cv::Vec3b>(y, x)[k] = rgb_data[y * 640 * 3 + x * 3 + k];
				}
			}
		}

		if (cvWaitKey(10) >= 0)
			break;

		this->onframe(rgb);
		// release frame
		psm->ReleaseFrame();
	}
	return 0;
}

int IntelRealsenseF200SDKRGB::close()
{
	cvReleaseImage(&image);
	psm->Release();
	return 0;
}

int IntelRealsenseF200SDKRGB::onframe(cv::Mat & image)
{
	this->cameraapi->onframe(image);
	return -1;
}

int IntelRealsenseF200SDKRGB::createProperties()
{
	// create properties
	std::string name = "IntelRealSenseF200RGB";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	return 0;
}

int IntelRealsenseF200SDKRGB::setProperties()
{
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	return 0;
}

int IntelRealsenseF200SDKRGB::setProperty(std::string name, boost::any value)
{
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power = boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b = psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	return -1;
}

IntelRealsenseF200SDKDepth::IntelRealsenseF200SDKDepth()
{
}

IntelRealsenseF200SDKDepth::IntelRealsenseF200SDKDepth(CameraAPI * cameraapi)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
}

IntelRealsenseF200SDKDepth::~IntelRealsenseF200SDKDepth()
{
}

int IntelRealsenseF200SDKDepth::init()
{

	std::cout << "Intel Realsense SDK Hacking using Opencv" << std::endl;
	std::cout << "Intel Realsense Camera SDK Frame Capture in opencv Mat Variable       -- by Deepak" << std::endl;
	std::cout << "Compiled with OpenCV version " << CV_VERSION << std::endl;

	psm = 0;
	psm = PXCSenseManager::CreateInstance();

	if (!psm) {
		wprintf_s(L"Unable to create the PXCSenseManager\n");
		//return 1;
	}

	psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480); //depth resolution
	psm->Init();

	// create Properties
	this->createProperties();

	// set Properties
	this->setProperties();

	///////////// OPENCV

	CvSize gab_size;
	gab_size.height = 480;
	gab_size.width = 640;

	depth = 0;
	CvSize gab_size_depth;
	gab_size_depth.height = 240;
	gab_size_depth.width = 320;
	depth = cvCreateImage(gab_size, 8, 1);

	PXCImage::ImageInfo rgb_info;
	PXCImage::ImageInfo depth_info;

	//cv::namedWindow("Mwindow", cv::WINDOW_AUTOSIZE);
	return 0;
}

int IntelRealsenseF200SDKDepth::aquistion()
{
	PXCImage::ImageData data_depth;

	//CircBuffer<cv::Mat> cb(10);

	for (;;)
	{
		if (psm->AcquireFrame(true)<PXC_STATUS_NO_ERROR) break;

		PXCCapture::Sample *sample = psm->QuerySample();
		PXCImage *depthIm;

		// retrieve the image or frame by type from the sample
		depthIm = sample->depth;

		PXCImage *depth_image = depthIm;

		depth_image->AcquireAccess(PXCImage::ACCESS_READ_WRITE, &data_depth);

		// create depth image
		depth_data = (short*)data_depth.planes[0];
		cv::Mat depth(480, 640, CV_8UC1);
		for (int x = 0; x < 480; x++)
		{
			for (int y = 0; y < 640; y++)
			{
				depth.at<uchar>(x, y) = depth_data[x * 640 + y];
			}
		}
		cv::equalizeHist(depth, depth);

		if (cvWaitKey(10) >= 0)
			break;

		this->onframe(depth);

		// release frame
		psm->ReleaseFrame();
	}
	return 0;
}

int IntelRealsenseF200SDKDepth::close()
{
	cvReleaseImage(&depth);
	psm->Release();
	return 0;
}

int IntelRealsenseF200SDKDepth::onframe(cv::Mat & image)
{
	this->cameraapi->onframe(image);
	return -1;
}

int IntelRealsenseF200SDKDepth::createProperties()
{
	// create properties
	std::string name = "IntelRealSenseF200";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	return 0;
}

int IntelRealsenseF200SDKDepth::setProperties()
{
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	return 0;
}

int IntelRealsenseF200SDKDepth::setProperty(std::string name, boost::any value)
{
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power = boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b = psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	return -1;
}
