#pragma once
#include "stdafx.h"
#include "Sphere.h"


Sphere::Sphere(void)
{
	m_radius = 0.02; 
}


Sphere::~Sphere(void)
{
	
}

void Sphere::init(double radius, Eigen::Vector3d* pos,  Eigen::Quaternion<double,Eigen::DontAlign>* orientation){

	if(pos) 
	m_position = *pos; 
	if(orientation)
	m_quat = (*orientation); 
	m_radius = radius; 

}


void Sphere::draw(){
	
	m_qobj = gluNewQuadric();
	glPushMatrix(); 
	transform(); 



	glMultMatrixf(m_matrix.constData());  
		
	gluQuadricDrawStyle(m_qobj, m_drawstyle);
	glColor3f(m_red, m_green, m_blue);
	gluSphere(m_qobj, m_radius, 20, 20);
		
	gluDeleteQuadric(m_qobj); 
	glPopMatrix();

}
