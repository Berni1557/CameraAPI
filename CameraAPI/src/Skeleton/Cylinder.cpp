/*
#pragma once
#include "stdafx.h"
#include <iostream>
#include "Cylinder.h"


Cylinder::Cylinder(void)
{
	m_radius = 0.02;
	m_height = 0.05;
	
}


Cylinder::~Cylinder(void)
{
}


void Cylinder::init(double radius, double height, Eigen::Vector3d* pos,  Eigen::Quaternion<double,Eigen::DontAlign>* orientation)
{
	if(pos)
		m_position = *pos; 

	if(orientation)
		m_quat = (*orientation); 	
	m_radius = radius; 
	m_height = height; 

	//red 
	m_red = 1.0; 
	m_green = 0.0; 
	m_blue= 0.0; 
}


void Cylinder::draw(){

		//std::cout << "IN" << std::endl;
		m_qobj = gluNewQuadric();
		glDisable(GL_LIGHTING);
		gluQuadricDrawStyle(m_qobj, m_drawstyle);
		gluQuadricNormals(m_qobj, GLU_NONE);
		gluQuadricOrientation(m_qobj,GLU_INSIDE);
		glColor3f(m_red, m_green, m_blue);
	
		glPushMatrix(); 

		
		transform(); 
		

		glMultMatrixf(m_matrix.constData());  
		
		//glRotatef(90, -1.0, 0.0, 0.0);
		gluCylinder(m_qobj, m_radius, m_radius, m_height, 12, 1);			
		gluDisk(m_qobj, 0.0, m_radius, 30, 1);
		
		gluQuadricOrientation(m_qobj,GLU_OUTSIDE);
		glTranslatef(0.0, 0.0, m_height);
		gluDisk(m_qobj,0.0, m_radius, 30, 1);	

		gluDeleteQuadric(m_qobj); 

		glPopMatrix(); 

}

*/