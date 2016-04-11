#pragma once
#ifndef SHAPE_H
#define SHAPE_H


#include "stdafx.h"
//#include "Headers.h"
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <stdio.h>
 #include <stdarg.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <vector>



/* OPENGL */
// GLEW

#include <glew.h>
#define GLEW_STATIC


// GLM 
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

//GLUT 
#include "glut.h"
#include <GL/gl.h>
#include <GL/glu.h> 


//QT 
#include <QtOpenGl\qglwidget> 
#include <QtGui\QMatrix4x4> 
#include <QtGui\QQuaternion>
#include <QTGui\QWheelevent> 
#include <QTCore\QTimer> 

//Eigen Boost
// Interface Arm A5
//#include "LeapToQ2.h"


// GLFW
#include <glfw3.h>

class Shape
{
	
public:
	GLUquadricObj *m_qobj;
	/*QVector3D m_position; 
	 QVector4D m_rotation; 
	 QQuaternion m_quat; */
	
	 Eigen::Quaternion<double,Eigen::DontAlign> m_quat;
	 Eigen::Vector3d m_position;
	 QMatrix4x4 m_matrix;
	
	GLfloat m_red, m_green, m_blue;
	GLenum m_drawstyle; 
	Shape(void);
	~Shape(void);

	virtual void init(){}; 
	virtual void draw(){}; 
	 
	void setColor(GLfloat red,  GLfloat green, GLfloat blue );
	 Eigen::Vector3d & getposition(void);
	void setDrawStyle(GLenum style);
	void transform(void);
};

#endif // SHAPE_H


