#pragma once

#pragma once

#ifndef A4LISTENER_H
#define A4LISTENER_H

#include <opencv2/core/core.hpp>
#include "ICamListener.h"
#include "QtCamListener.h"

#include <QtWidgets>
#include <QtGui/QImage>

class A4CamListener : public QtCamListener {
	// Operations
public:
	int onframe(QImage qimage);

};


#endif // A4LISTENER_H
