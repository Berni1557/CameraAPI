#include "ICamListener.h"

ICamListener::ICamListener()
{
}

ICamListener::~ICamListener()
{
}

int ICamListener::update(cv::Mat & image)
{
	this->image = image;
	return 0;
}
