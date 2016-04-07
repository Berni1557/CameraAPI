#include "ExtrinsicParameterB3.h"

ExtrinsicParameterB3::ExtrinsicParameterB3(cv::Vec3d translationVector_default, cv::Mat3d rotationMatrix_default)
{
	this->translationVector_default = translationVector_default;
	this->rotationMatrix_default = rotationMatrix_default;
}

int ExtrinsicParameterB3::computeParameter(cv::Mat image, cv::Mat cameraMatrix)
{
	this->findCalibrationCircles(image);
	return 0;
}

int ExtrinsicParameterB3::findCalibrationCircles(cv::Mat image)
{
	return 0;
}
