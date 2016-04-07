#include "ExtrinsicParameter.h"

ExtrinsicParameter::ExtrinsicParameter()
{
	this->translationVector[0] = 0;
	this->translationVector[1] = 0;
	this->translationVector[2] = 0;
	this->rotationMatrix = cv::Mat3d::eye(3,3);
}

ExtrinsicParameter::~ExtrinsicParameter()
{
}
