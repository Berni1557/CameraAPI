#pragma once
#ifndef CYLINDER_H
#define CYLINDER_H

#include "Shape.h"
class Cylinder : public Shape
{
public:
	Cylinder(void);
	~Cylinder(void);

	double m_radius;
	double m_height;

	void init(double radius, double height,Eigen::Vector3d* pos=NULL,  Eigen::Quaternion<double,Eigen::DontAlign>* orientation=NULL );
	void draw(); 
};

#endif // end CYLINDER_H



