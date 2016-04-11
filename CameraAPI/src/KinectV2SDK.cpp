#include "KinectV2SDK.h"

inline void CHECKERROR(HRESULT);
template<class Interface> inline void SAFERELEASE(Interface *&);
template<class Interface> inline void SafeReleaseS(Interface *& pInterfaceToRelease);

KinectV2SDK::KinectV2SDK()
{
}

KinectV2SDK::KinectV2SDK(CameraAPI* cameraapi, KinectV2SDKRGB* icameraRGB, KinectV2SDKDepth* icameraDepth)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
	this->icameraRGB = icameraRGB;
	this->icameraDepth = icameraDepth;
}

KinectV2SDK::~KinectV2SDK()
{
}

int KinectV2SDK::init()
{
	// Kinect V2 to OpenCV

	HRESULT hr;
	kinectSensor = nullptr;     // kinect sensor

											   // initialize Kinect Sensor
	hr = GetDefaultKinectSensor(&kinectSensor);
	if (FAILED(hr) || !kinectSensor) {
		std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
		return -1;
	}
	CHECKERROR(kinectSensor->Open());

	// initialize color frame reader
	IColorFrameSource* colorFrameSource = nullptr; // color source
	CHECKERROR(kinectSensor->get_ColorFrameSource(&colorFrameSource));
	CHECKERROR(colorFrameSource->OpenReader(&colorFrameReader));
	SAFERELEASE(colorFrameSource);

	return 0;
}


int KinectV2SDK::aquistion()
{
	cv::Mat rgb;
	int m;
	while (colorFrameReader) {
		m=colorFrameReaderToMat(rgb);
		bool k = kbhit();
		int key = cv::waitKey(1000);
		if (k) {
			break;
		}
		if (m == 0) {
			//cv::imshow("Body", rgb);
			//cv::waitKey();
			//this->drawSkeleton(rgb);

			this->icameraRGB->onframe(rgb);

		}
	}
	return 0;
}

int KinectV2SDK::close()
{
	// de-initialize Kinect Sensor
	CHECKERROR(kinectSensor->Close());
	SAFERELEASE(kinectSensor);
	return 0;
}

int KinectV2SDK::createProperties()
{
	/*
	// create properties
	std::string name = "IntelRealSenseF200";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	*/
	return 0;
}

int KinectV2SDK::setProperties()
{
	/*
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	*/
	return 0;
}

int KinectV2SDK::setProperty(std::string name, boost::any value)
{
	/*
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power = boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b = psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	*/
	return -1;
}

int KinectV2SDK::getCalibIntelRealsenseF200()
{
	/*
	psm = 0;
	psm = PXCSenseManager::CreateInstance();
	PXCCaptureManager* cm = psm->QueryCaptureManager();
	PXCCapture::Device* d = cm->QueryDevice();
	PXCCalibration *calib = psm->QueryCaptureManager()->QueryDevice()->CreateProjection()->QueryInstance<PXCCalibration>();


	PXCCalibration::StreamCalibration* scalib;
	PXCCalibration::StreamTransform *transformation;
	pxcStatus st = calib->QueryStreamProjectionParameters(PXCCapture::StreamType::STREAM_TYPE_COLOR, scalib, transformation);

	PXCPointF32 p = scalib->focalLength;
	*/
	return 0;
}


int KinectV2SDK::drawSkeleton(cv::Mat& image)
{
	HRESULT hResult = S_OK;
	int width = 0;
	int height = 0;
	cv::Mat skeleton;

	cv::Point SpineBase;
	cv::Point SpineMid;
	cv::Point Neck;
	cv::Point Head;
	cv::Point ShoulderLeft;
	cv::Point ElbowLeft;
	cv::Point WristLeft;
	cv::Point HandLeft;
	cv::Point ShoulderRight;
	cv::Point ElbowRight;
	cv::Point WristRight;
	cv::Point HandRight;
	cv::Point HipLeft;
	cv::Point HipRight;
	cv::Point SpineShoulder;
	cv::Point HandTipLeft;
	cv::Point ThumbLeft;
	cv::Point HandTipRight;
	cv::Point ThumbRight;

	IKinectSensor* pSensor = kinectSensor;

	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
	}

	// Source
	IColorFrameSource* pColorSource;
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
	}

	IBodyFrameSource* pBodySource;
	hResult = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
	}

	// Reader
	IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
	}

	IBodyFrameReader* pBodyReader;
	hResult = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
	}

	// Description
	IFrameDescription* pDescription;
	hResult = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
	}

	pDescription->get_Width(&width); // 1920
	pDescription->get_Height(&height); // 1080
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

	//cv::Mat bufferMat(height, width, CV_8UC3);
	cv::Mat bufferMat(image);
	cv::imshow("Body", image);

	cv::waitKey();

	//cvtColor(image, bufferMat, CV_BGR2BGRA);


	cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
	cv::namedWindow("Body");

	// Color Table
	cv::Vec3b color[BODY_COUNT];
	color[0] = cv::Vec3b(255, 0, 0);
	color[1] = cv::Vec3b(0, 255, 0);
	color[2] = cv::Vec3b(0, 0, 255);
	color[3] = cv::Vec3b(255, 255, 0);
	color[4] = cv::Vec3b(255, 0, 255);
	color[5] = cv::Vec3b(0, 255, 255);

	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;

	}


	while (1) {
		// Frame
		IColorFrame* pColorFrame = nullptr;
		hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hResult)) {
			//hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
			if (SUCCEEDED(hResult)) {
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
		}
		//SafeRelease( pColorFrame );

		IBodyFrame* pBodyFrame = nullptr;
		hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hResult)) {
			IBody* pBody[BODY_COUNT] = { 0 };
			hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
			if (SUCCEEDED(hResult)) {
				for (int count = 0; count < BODY_COUNT; count++) {
					BOOLEAN bTracked = false;
					hResult = pBody[count]->get_IsTracked(&bTracked);
					if (SUCCEEDED(hResult) && bTracked) {
						Joint joint[JointType::JointType_Count];
						hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
						if (SUCCEEDED(hResult)) {
							int x_joint[25];
							int y_joint[25];
							// Joint
							for (int type = 0; type < JointType::JointType_Count; type++) {
								if (type == 13 || type == 14 || type == 15 || type == 17 || type == 18 || type == 19) {
									//joints of legs not needed for upper body
								}
								else {
									ColorSpacePoint colorSpacePoint = { 0 };
									pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
									int x = static_cast<int>(colorSpacePoint.X);
									int y = static_cast<int>(colorSpacePoint.Y);
									x_joint[type] = x;
									y_joint[type] = y;
									if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
										cv::circle(bufferMat, cv::Point(x, y), 12, static_cast< cv::Scalar >(color[count]), -1, CV_AA);
									}
								}
							}



							//Converting coordinates to cv::Point
							SpineBase = cv::Point(x_joint[0], y_joint[0]);
							SpineMid = cv::Point(x_joint[1], y_joint[1]);
							Neck = cv::Point(x_joint[2], y_joint[2]);
							Head = cv::Point(x_joint[3], y_joint[3]);
							ShoulderLeft = cv::Point(x_joint[4], y_joint[4]);
							ElbowLeft = cv::Point(x_joint[5], y_joint[5]);
							WristLeft = cv::Point(x_joint[6], y_joint[6]);
							HandLeft = cv::Point(x_joint[7], y_joint[7]);
							ShoulderRight = cv::Point(x_joint[8], y_joint[8]);
							ElbowRight = cv::Point(x_joint[9], y_joint[9]);
							WristRight = cv::Point(x_joint[10], y_joint[10]);
							HandRight = cv::Point(x_joint[11], y_joint[11]);
							HipLeft = cv::Point(x_joint[12], y_joint[12]);
							HipRight = cv::Point(x_joint[16], y_joint[16]);
							SpineShoulder = cv::Point(x_joint[20], y_joint[20]);
							HandTipLeft = cv::Point(x_joint[21], y_joint[21]);
							ThumbLeft = cv::Point(x_joint[22], y_joint[22]);
							HandTipRight = cv::Point(x_joint[23], y_joint[23]);
							ThumbRight = cv::Point(x_joint[24], y_joint[24]);



							//Draw the Torso
							cv::line(bufferMat, Neck, Head, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, Neck, SpineShoulder, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineMid, SpineShoulder, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineMid, SpineBase, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineShoulder, ShoulderRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineShoulder, ShoulderLeft, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineBase, HipRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, SpineBase, HipLeft, static_cast< cv::Scalar >(color[count]), 5);



							// Draw Right Arm
							cv::line(bufferMat, ShoulderRight, ElbowRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, ElbowRight, WristRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, WristRight, HandRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, HandRight, HandTipRight, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, WristRight, ThumbRight, static_cast< cv::Scalar >(color[count]), 5);

							// Draw Left Arm
							cv::line(bufferMat, ShoulderLeft, ElbowLeft, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, ElbowLeft, WristLeft, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, WristLeft, HandLeft, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, HandLeft, HandTipLeft, static_cast< cv::Scalar >(color[count]), 5);
							cv::line(bufferMat, WristLeft, ThumbLeft, static_cast< cv::Scalar >(color[count]), 5);

						}


						// Lean
						PointF amount;
						hResult = pBody[count]->get_Lean(&amount);
						if (SUCCEEDED(hResult)) {
							std::cout << "amount : " << amount.X << ", " << amount.Y << std::endl;
						}
					}
				}
				cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
			}
			for (int count = 0; count < BODY_COUNT; count++) {
				SafeReleaseS(pBody[count]);
			}
		}
		SafeReleaseS(pColorFrame);
		SafeReleaseS(pBodyFrame);

		cv::imshow("Body", bodyMat);

		if (cv::waitKey(10) == VK_ESCAPE) {
			break;
		}
	}
	return 0;
}


KinectV2SDKRGB::KinectV2SDKRGB()
{
}

KinectV2SDKRGB::KinectV2SDKRGB(CameraAPI * cameraapi)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
}

KinectV2SDKRGB::~KinectV2SDKRGB()
{
}

int KinectV2SDKRGB::init()
{
	// Kinect V2 to OpenCV

	HRESULT hr;
	kinectSensor = nullptr;     // kinect sensor

								// initialize Kinect Sensor
	hr = GetDefaultKinectSensor(&kinectSensor);
	if (FAILED(hr) || !kinectSensor) {
		std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
		return -1;
	}
	CHECKERROR(kinectSensor->Open());

	// initialize color frame reader
	IColorFrameSource* colorFrameSource = nullptr; // color source
	CHECKERROR(kinectSensor->get_ColorFrameSource(&colorFrameSource));
	CHECKERROR(colorFrameSource->OpenReader(&colorFrameReader));
	SAFERELEASE(colorFrameSource);
	return 0;
}

int KinectV2SDKRGB::aquistion()
{
	/*
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
	*/
	return 0;
}

int KinectV2SDKRGB::close()
{
	/*
	cvReleaseImage(&image);
	psm->Release();
	*/
	return 0;
}

int KinectV2SDKRGB::onframe(cv::Mat & image)
{
	//IntrinsicParameter IP;
	//this->getSDKCalibrationModel(IP);

	return this->cameraapi->onframe(image);
}

int KinectV2SDKRGB::createProperties()
{
	// create properties
	std::string name = "IntelRealSenseF200RGB";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	return 0;
}

int KinectV2SDKRGB::setProperties()
{
	/*
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	*/
	return 0;
}

int KinectV2SDKRGB::setProperty(std::string name, boost::any value)
{
	/*
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power = boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b = psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	*/
	return -1;
}


KinectV2SDKDepth::KinectV2SDKDepth()
{
}

KinectV2SDKDepth::KinectV2SDKDepth(CameraAPI * cameraapi)
{
	this->cameraapi = cameraapi;
	this->camProperties = CamProperties();
}

KinectV2SDKDepth::~KinectV2SDKDepth()
{
}

int KinectV2SDKDepth::init()
{
	// Kinect V2 to OpenCV

	HRESULT hr;
	kinectSensor = nullptr;     // kinect sensor

								// initialize Kinect Sensor
	hr = GetDefaultKinectSensor(&kinectSensor);
	if (FAILED(hr) || !kinectSensor) {
		std::cout << "ERROR hr=" << hr << "; sensor=" << kinectSensor << std::endl;
		return -1;
	}
	CHECKERROR(kinectSensor->Open());

	// initialize color frame reader
	IDepthFrameSource* depthFrameSource = nullptr; // color source
	CHECKERROR(kinectSensor->get_DepthFrameSource(&depthFrameSource));
	CHECKERROR(depthFrameSource->OpenReader(&depthFrameReader));
	SAFERELEASE(depthFrameSource);
	return 0;
}

int KinectV2SDKDepth::aquistion()
{
	/*
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
	*/
	return 0;
}

int KinectV2SDKDepth::close()
{
	/*
	cvReleaseImage(&depth);
	psm->Release();
	*/
	return 0;
}

int KinectV2SDKDepth::onframe(cv::Mat & image)
{
	this->cameraapi->onframe(image);
	return -1;
}

int KinectV2SDKDepth::createProperties()
{
	/*
	// create properties
	std::string name = "IntelRealSenseF200";
	this->camProperties.createProperty("Cameraname", name);
	this->camProperties.createProperty("IVCAMLaserPower", 16.0);
	*/
	return 0;
}

int KinectV2SDKDepth::setProperties()
{
	/*
	// set laser power
	double power;
	this->camProperties.getProperty("IVCAMLaserPower", power);
	psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
	*/
	return 0;
}

int KinectV2SDKDepth::setProperty(std::string name, boost::any value)
{
	/*
	if (name.compare("IVCAMLaserPower") == 0) {
		// set laser power
		double power = boost::any_cast<double>(value);
		psm->QueryCaptureManager()->QueryDevice()->SetIVCAMLaserPower((pxcF32)power);
		pxcBool b = psm->QueryCaptureManager()->QueryDevice()->QueryColorAutoExposure();

		return 0;
	}
	*/
	return -1;
}

int KinectV2SDKDepth::getSDKCalibrationModel(IntrinsicParameter & intrinsic)
{
	HRESULT hResult = S_OK;
	// Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper;
	hResult = kinectSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
	}
	CameraIntrinsics* CI = new CameraIntrinsics();
	hResult = pCoordinateMapper->GetDepthCameraIntrinsics(CI);
	if (FAILED(hResult)) {
		std::cerr << "Error : pCoordinateMapper::GetDepthCameraIntrinsics()" << std::endl;
	}

	if (CI->FocalLengthX != 0) {
		intrinsic.focalLengthX = CI->FocalLengthX;
		intrinsic.focalLengthY = CI->FocalLengthY;
		intrinsic.opticalCenterX = CI->PrincipalPointX;
		intrinsic.opticalCenterY = CI->PrincipalPointY;
		return 0;
	}
	else {
		return -1;
	}

}


inline void CHECKERROR(HRESULT n) {
	if (!SUCCEEDED(n)) {
		std::stringstream ss;
		ss << "ERROR " << std::hex << n << std::endl;
		std::cin.ignore();
		std::cin.get();
		throw std::runtime_error(ss.str().c_str());
	}
}

// Safe release for interfaces
template<class Interface> inline void SAFERELEASE(Interface *& pInterfaceToRelease) {
	if (pInterfaceToRelease != nullptr) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

int KinectV2SDK::colorFrameReaderToMat(cv::Mat& rgb) {
	IColorFrame *data = nullptr;
	IFrameDescription *frameDesc = nullptr;
	HRESULT hr = -1;
	RGBQUAD *colorBuffer = nullptr;
	int m=-1;

	hr = colorFrameReader->AcquireLatestFrame(&data);
	if (SUCCEEDED(hr)) hr = data->get_FrameDescription(&frameDesc);
	if (SUCCEEDED(hr)) {
		int height = 0, width = 0;
		if (SUCCEEDED(frameDesc->get_Height(&height)) &&
			SUCCEEDED(frameDesc->get_Width(&width))) {
			colorBuffer = new RGBQUAD[height * width];
			hr = data->CopyConvertedFrameDataToArray(height * width * sizeof(RGBQUAD),
				reinterpret_cast<BYTE*>(colorBuffer), ColorImageFormat_Bgra);
			if (SUCCEEDED(hr)) {
				cv::Mat image(height, width, CV_8UC4, reinterpret_cast<void*>(colorBuffer));
				image.copyTo(rgb);
				//cv::imshow("Color Only", image);
				m = 0;
			}
		}
	}
	if (colorBuffer != nullptr) {
		delete[] colorBuffer;
		colorBuffer = nullptr;
	}
	SAFERELEASE(data);
	return m;
}

template<class Interface> inline void SafeReleaseS(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}