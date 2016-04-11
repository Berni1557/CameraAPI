#ifndef SPHERE_H
#define SPHERE_H


#include "Shape.h"

class Sphere :public Shape
{
public:
	Sphere(void);
	~Sphere(void);
	
	double m_radius; 
	void init(double radius, Eigen::Vector3d* pos =NULL,  Eigen::Quaternion<double,Eigen::DontAlign>* orientation=NULL); 
	void draw(); 
};

#endif // end SPHERE_H

