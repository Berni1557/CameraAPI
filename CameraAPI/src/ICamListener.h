#pragma once

#ifndef ICAMLISTENER_H
#define ICAMLISTENER_H

#include <opencv2/core/core.hpp>

class ICamListener{
	// Operations
public:
	ICamListener();
	~ICamListener();
	virtual int update(cv::Mat& image);

	// Attributes
private:
	cv::Mat image;

};





#endif // ICAMLISTENER_H