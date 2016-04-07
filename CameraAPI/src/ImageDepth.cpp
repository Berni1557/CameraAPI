#include "ImageDepth.h"

ImageDepth::ImageDepth()
{
}

ImageDepth::ImageDepth(cv::Mat& image)
{
	this->imageDepth = image;
}

ImageDepth::~ImageDepth()
{
}

Imsize ImageDepth::size()
{
	return Imsize(imageDepth.size().width, imageDepth.size().height);
}

cv::Mat & ImageDepth::image()
{
	return imageDepth;
}
