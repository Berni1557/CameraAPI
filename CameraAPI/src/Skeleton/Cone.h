#pragma once
#ifndef CONE_H
#define CONE_H

#include "Shape.h"
class Cone :public Shape
{
public:
	Cone(void);
	~Cone(void);

	double m_radius;
	double m_height;

	void init(double radius, double height,Eigen::Vector3d* pos=NULL,  Eigen::Quaternion<double,Eigen::DontAlign>* orientation=NULL );
	void draw(); 
};

#endif  // end CONE_H



