/*
#include "stdafx.h"
#include "GLWidget.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std; 
const double GLWidget::wheelSpeed = 1.0 / 120 / 8; // 120 = value of one step, 8 steps to double/half the value
int frameNo = 0;
GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(parent)
{
	setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
	bc_red = bc_green = bc_blue = 0.0f;
	stop = false; 
	cameraReset();

	tMode = Rotation;
}


GLWidget::~GLWidget(void)
{
	

}


void GLWidget::initializeGL(void)
{
	glShadeModel(GL_SMOOTH);
	glClearColor(bc_red, bc_green, bc_blue, 1.0f);
	

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	//glEnable(GL_CULL_FACE);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

}

static float qNormalizeAngle(float angle)
{
	while (angle < -180)
		angle += 360 ;
	while (angle > 180)
		angle -= 360;
	return angle;
}

static void qNormalizeRotation(QVector3D &rotation)
{
	rotation.setX(qNormalizeAngle(rotation.x()));
	rotation.setY(qNormalizeAngle(rotation.y()));
	rotation.setZ(qNormalizeAngle(rotation.z()));
}


void GLWidget::paintGL(void)
{

	glClearColor(bc_red, bc_green, bc_blue, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glViewport(0, 0, width(), height());
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float cameraPoseAngle = 45.0f;
	GLfloat x = GLfloat(width()) / height();
	gluPerspective(cameraPoseAngle, x, 0.1f, 100.0f);
	//qDebug() << zoom; 

	draw(); 
	
	

}

void GLWidget::draw(void)
{

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	
	//qDebug() << cameraMatrix; 
	glMultMatrixf(cameraMatrix.constData());
	
	
	//draw all joints and bones (initialisation: --> initHand(); ) 
	//joint_WristCMC.draw();
	//joint_MiddleTCP.draw();
	//joint_Elbow.draw();

//	cylinder_WirstMiddle.draw(); 
	//cylinder_ElbowWrist.draw(); 
	//cone1.draw(); 


	this->S.draw();


	drawAxes();





}



void GLWidget::resizeGL(int w, int h)
{

	if (this->height()==0)
	{
		this->resize(this->width(), 1);
	}

	glViewport(0, 0, w, h);
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float cameraPoseAngle = 45.0f;
	GLfloat x = GLfloat(w) / h;
	gluPerspective(cameraPoseAngle, x, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	updateGL();

}


void GLWidget::updateWidget(void)
{
	updateGL(); 
}





void GLWidget::changeTMode_Translation()
{
	this->tMode = Translation;
}

void GLWidget::changeTMode_Rotation()
{
	this->tMode = Rotation;
}

void GLWidget::changeTMode_Zoom()
{
	this->tMode = Zoom;
}

void GLWidget::changeTMode_Select()
{
	this->tMode = Select;
}



void GLWidget::keyPressEvent(QKeyEvent *e)
{
	switch(e->key()){
	case Qt::Key::Key_Escape:
		close();
		break; 
	case Qt::Key::Key_R:{
		qDebug() << "camera Reset";
			cameraReset(); 
			break; }

	case Qt::Key::Key_Left:{
		qDebug() << "camera translate:" ;
			rotateCamera(-1, 0); 
			break; }

	case Qt::Key::Key_Right:{
		qDebug() << "camera translate:" ;
			rotateCamera(1, 0); 
			break; }

	case Qt::Key::Key_Down:{
		qDebug() << "camera translate:" ;
			rotateCamera(0, -1); 
			break; }

	case Qt::Key::Key_Up:{
		qDebug() << "camera translate:" ;
			rotateCamera(0, 1); 
			break; }

	case Qt::Key::Key_Space:{
			stop = !stop;
			if(stop)
				qDebug() << "Stop"; 
			else
				qDebug() <<"Run"; 
			break; }

	default: 
		break; 
		
		
	}
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
	switch(event->button()){
	case Qt::LeftButton:
		changeTMode_Translation();
		break;
	case Qt::RightButton:
		changeTMode_Rotation();
		break;
	}
    clickPosition = event->pos();
	firstClickPosition = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{

	if(clickPosition.x() == -1 && clickPosition.y() == -1)
		return;

    double deltaX = double(event->x() - clickPosition.x()) ;
    double deltaY = double(event->y() - clickPosition.y()) ;
  
	switch(tMode)
	{
		case Translation:
			translateCamera(deltaX, deltaY);
			break;
		case Rotation:
			rotateCamera(deltaX, deltaY);
			break;
		case Zoom:
			zoomCamera(deltaY);
			break;
	//	case Select:
		//	selectAt(clickPosition);
	}
	clickPosition = event->pos();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
	clickPosition.setX(-1);
	clickPosition.setY(-1);
}

void GLWidget::wheelEvent(QWheelEvent *event)
{
	
	zoomCamera(-(event->delta()));
	updateGL();
}

void GLWidget::zoomCamera(double deltaZoom)
{
	setZoom(zoom + zoom * deltaZoom * wheelSpeed);
	qDebug() << "zoom" << this << ": " << zoom << ", deltaZoom: " << deltaZoom;
	update();
}



void GLWidget::setZoom(double zoom){
	this->zoom = zoom;
	if (zoom < 0.1){
		this->zoom = 0.1;
	}
}

void GLWidget::setRotation(QVector3D newRotation)
{
	rotation = newRotation;
	qNormalizeRotation(rotation);
	qDebug() << "setRotation x: " << rotation.x() << ", y: " << rotation.y() << ", z: " << rotation.z();
	update();
}



//Camera 
void GLWidget::update()
{
	
	cameraMatrix.setToIdentity();
//	if (mode == NORMAL){
		//change rotation so z=UpVector (order: x-y-z)
	//	cameraMatrix.rotate(-90,1,0,0);
	//	cameraMatrix.rotate(90,0,0,1);
	//	QQuaternion quat (0.5,-0.5,0.5,0.5);	
	//	cameraMatrix.rotate(quat); 
		QQuaternion quat (1,0,0,0);	
		cameraMatrix.rotate(quat); 
		cameraMatrix.translate(0,0,-zoom);
		
//	}
// else {
//		cameraMatrix.translate(0,0,-zoom);
//		cameraMatrix.rotate(180,0,1,0);
//	}
	cameraMatrix.rotate(rotation.x(),1,0,0);
	cameraMatrix.rotate(rotation.y(),0,1,0);
	cameraMatrix.rotate(rotation.z(),0,0,1);
	cameraMatrix.translate(focusPosition);
//	emit viewPopertiesChanged(rotation,focusPosition,zoom);
	updateGL();
}


void GLWidget::translateCamera(double deltaX, double deltaY)
{
	QMatrix4x4 matrix;
	matrix.rotate(rotation.x(),1,0,0);
	matrix.rotate(rotation.y(),0,1,0);
	matrix.rotate(rotation.z(),0,0,1);
	
	matrix.translate(deltaX * 0.01,-deltaY * 0.01,0);
	
	focusPosition += QVector3D(matrix.data()[12],matrix.data()[13],matrix.data()[14]);
	qDebug() << "Translate x: " << focusPosition.x() << ", y: " << focusPosition.y() << ", z: " << focusPosition.z();
	update();
}

void GLWidget::rotateCamera(double angleYaw, double anglePitch)
{

	rotation += QVector3D(-anglePitch * 0.25,angleYaw * 0.25,0);
	
	qNormalizeRotation(rotation);
	qDebug() << "Rotate x: " << rotation.x() << ", y: " << rotation.y() << ", z: " << rotation.z();
	update();
}

void GLWidget::cameraReset()
{
	zoom = 1;
	rotation = QVector3D();
	focusPosition = QVector3D();
	update();
}




QVector3D &GLWidget::getRotation(){
	return rotation;
}

QVector3D &GLWidget::getFocusPosition(){
	return focusPosition;
}

//getters 
double GLWidget::getZoom() const{
	return zoom;
}


void GLWidget::drawAxes(void)
{
	glPushMatrix(); 
	//draw axes
	glBegin(GL_LINES);
	glColor3f(1.0,0,0);
	glVertex3d(0,0,0);
	glVertex3d(5,0,0);

	glColor3f(0,1.0,0);
	glVertex3d(0,0,0);
	glVertex3d(0,5,0);

	glColor3f(0,0,1.0);
	glVertex3d(0,0,0);
	glVertex3d(0,0,5);
	glEnd();

	glPopMatrix(); 
	

}

bool GLWidget::initHand(Q2::ArmA5& arm)
{

//Press "space" key to freeze/run 
	if(!stop){
		


		double length_hand = arm.getMiddlehand().getBoneLength();
		double  length_forearm = arm.getForearm().getBoneLength(); 

		joint_WristCMC.init(0.02, &arm.wristPosition(), &arm.getMiddlehand().getQuaternionUnion()); 
		joint_MiddleTCP.init(0.01, &arm.middelhandPosition()); 
		
		joint_Elbow.init(0.01, &arm.elbowPosition(), &arm.getForearm().getQuaternionUnion()); 
	 
		cylinder_WirstMiddle.init(0.01, length_hand,&arm.wristPosition(), &arm.getMiddlehand().getQuaternionUnion()); 
		cylinder_ElbowWrist.init(0.01, length_forearm,&arm.elbowPosition(), &arm.getForearm().getQuaternionUnion());

		cone1.init(0.01, length_hand,&arm.wristPosition(), &arm.getMiddlehand().getQuaternionUnion()); 



		return true;	}

	else
		return false; 
	
}

bool initSkeleton(Q2::Skeleton& S);

bool GLWidget::initSkeleton(Q2::Skeleton& Sin)
{

	this->S.setGraph(Sin.getGraph());
	this->S.init(Q2::sphere, Q2::cylinder);
	return true;
}


*/