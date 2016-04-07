#include "ImageRGB.h"

Imsize ImageRGB::size()
{
	return Imsize(imageRGB.size().width, imageRGB.size().height);
}

cv::Mat & ImageRGB::image()
{
	return imageRGB;
}

ImageRGB::ImageRGB()
{
}

ImageRGB::ImageRGB(cv::Mat & image)
{
	this->imageRGB = image;
}

ImageRGB::~ImageRGB()
{
}
