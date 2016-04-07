#include "ImageGray.h"

ImageGray::ImageGray()
{
}

ImageGray::~ImageGray()
{
}

Imsize ImageGray::size()
{
	return Imsize(imageGray.size().width, imageGray.size().height);
}

cv::Mat & ImageGray::image()
{
	return imageGray;
}
