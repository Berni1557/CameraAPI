#pragma once
#include "stdafx.h"
#include "Shape.h"



Shape::Shape(void)
{
	
	m_position[0] = 0.0; 
	m_position[1] = 0.0; 
	m_position[2] = 0.0; 
	
	m_quat = Eigen::Quaternion<double,Eigen::DontAlign> (1,0,0,0);  

	//Default Color: green 
	m_red = 0.0; 
	m_green = 1.0; 
	m_blue = 0.0; 
	m_drawstyle = GLU_LINE; 
}


Shape::~Shape(void)
{
	
}


void Shape::setColor(GLfloat red,  GLfloat green, GLfloat blue )
{
	m_red = red; 
	m_green = green; 
	m_blue = blue; 
}


Eigen::Vector3d & Shape::getposition(void)
{
	return m_position; 
}

// e.g. GLU_LINE, GLU_SOLID
void Shape::setDrawStyle(GLenum style)
{

	m_drawstyle = style; 
}

void Shape::transform(void)
{

	//Rotation quaternions
	//OpenGl -> Local
	Eigen::Matrix3d M_OLL = Eigen::Matrix3d::Zero();
		M_OLL(0,2) = 1; 
		M_OLL(1,0) = 1; 
		M_OLL(2,1) = 1; 
	//OpenGl->Global
	Eigen::Matrix3d M_OG = Eigen::Matrix3d::Zero();
		M_OG(0,2) = -1; 
		M_OG(1,0) = -1; 
		M_OG(2,1) = 1; 
	//Matrix to Quaternion 
	Eigen::Quaternion<double,Eigen::DontAlign> q_OLL; 
		q_OLL=Eigen::Quaternion<double,Eigen::DontAlign>(1,0,0,0);
		q_OLL = M_OLL; 

	Eigen::Quaternion<double,Eigen::DontAlign> q_OG; 
		q_OG=Eigen::Quaternion<double,Eigen::DontAlign>(1,0,0,0);
		q_OG = M_OG; 

	//OLocal -> OpenGl 
	Eigen::Matrix3d M_OLO = Eigen::Matrix3d::Zero();
	Eigen::Quaternion<double,Eigen::DontAlign> q_OLO; 
	q_OLO = q_OG.inverse() * m_quat * q_OLL; 
	M_OLO = q_OLO; 

	m_matrix.setToIdentity();
	
	m_matrix.translate(-m_position.y(), m_position.z() , -m_position.x());
	


	QQuaternion quat(q_OLO.w(), q_OLO.x(), q_OLO.y(), q_OLO.z()); 
	m_matrix.rotate(quat); 
	//m_matrix.rotate(m_rotation.x(),1,0,0);
	//m_matrix.rotate(m_rotation.y(),0,1,0);
	//m_matrix.rotate(m_rotation.z(),0,0,1);
	

}
