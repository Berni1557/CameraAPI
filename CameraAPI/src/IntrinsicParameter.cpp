#include "IntrinsicParameter.h"

IntrinsicParameter::IntrinsicParameter()
{
	focalLengthX=0;
	focalLengthY = 0;
	opticalCenterX = 0;
	opticalCenterY = 0;
	aspectRatio = 0;

	focalLengthXDefault = 0;
	focalLengthYDefault = 0;
	opticalCenterXDefault = 0;
	opticalCenterYDefault = 0;
	aspectRatioDefault = 0;
}

IntrinsicParameter::~IntrinsicParameter()
{
}
