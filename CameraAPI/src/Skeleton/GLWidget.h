/*
#ifndef GLWIDGET
#define GLWIDGET 

#include "Headers.h"
#include "Sphere.h" 
#include "Cylinder.h" 
#include "Cone.h" 
#include "ArmA5.h" 
#include "Skeleton.h" 

#define PI  3.14159265358979323846f 
class GLWidget: public QGLWidget
{
	//Q_OBJECT
public:
	GLWidget(QWidget *parent = 0);
	~GLWidget(void);

protected:
	void initializeGL(void);
	void paintGL(void);
	void resizeGL(int w, int h);
		void keyPressEvent(QKeyEvent*);


public:	
	//draw function 
	void draw(void);
	//mouse functions 
	void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
   //zoom functions 
	void setZoom(double zoom);
	void wheelEvent(QWheelEvent *event);

	//camera functions 
	void cameraReset();
	void translateCamera(double deltaX, double deltaY);
	void zoomCamera(double deltaZoom);
	void rotateCamera(double angleYaw, double anglePitch);
	void setRotation(QVector3D newRotation);

	//getters 
	QVector3D &getRotation();
	QVector3D &getFocusPosition();
	double getZoom() const;
private: 
	GLboolean stop; 
	GLfloat bc_red, bc_green, bc_blue;

	//Modes
	enum TransformationMode{
        Translation,
        Rotation,
        Zoom,
		Select
    };
	TransformationMode tMode;

	void changeTMode_Translation(void);
	void changeTMode_Rotation(void);
	void changeTMode_Zoom(void);
	void changeTMode_Select(void);

	//camera (zoom, mouse) 
	QMatrix4x4 cameraMatrix;
	QPoint clickPosition;
	QPoint firstClickPosition;
	QVector3D focusPosition;
	QVector3D rotation;
	double zoom;
	static const double wheelSpeed;
	
	
	//update functions 
	
	
	void updateWidget(void);
public:
	void update();

	Q2::Skeleton S;

	Sphere joint_WristCMC;
	Sphere joint_MiddleTCP;
	Sphere joint_Shoulder; 
	Sphere joint_Elbow; 
	Sphere testsphere; 
	Cylinder cylinder_WirstMiddle, cylinder_ElbowWrist, cylinder_ShoulderElbow;
	Cone cone1; 
	void readFile(); 
		
	void drawAxes(void);
	bool initHand(Q2::ArmA5& arm);
	bool initSkeleton(Q2::Skeleton& S);
};

#endif GLWIDGET 
*/