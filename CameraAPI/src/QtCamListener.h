#pragma once

#ifndef CAMLISTENER_H
#define CAMLISTENER_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ICamListener.h"

#include <QtWidgets>
#include <QtGui/QImage>

class QtCamListener : public ICamListener {
	// Operations
public:
	virtual int onframe(QImage qimage)=0;
	int update(cv::Mat& image);
	QImage MatToQImage(const cv::Mat& mat);

private:
	QImage qimage;
};





#endif // CAMLISTENER_H
