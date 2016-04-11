/**
@file ArmA5.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/

#include <iostream>
#include <list>
#include <boost/math/constants/constants.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "Leap.h"
#include "LeapToQ2.h"
#include <Skeleton.h>
#include "ArmA5.h"
#include "Hand.h"

const double PI = boost::math::constants::pi<double>();

namespace Q2{

	Hand::Hand(){
	};

	Hand::Hand(bool isRight){
		this->isRight=isRight;
		this->isLeft=!isRight;
	};

	Hand::Hand(Pinky pinky, Ring ring, Middle middle, Index index, Thumb thumb){
		JointVertex v1=0;
		JointVertex v2=0;
		Eigen::Matrix3d RotMat = Eigen::Matrix3d::Identity();
		//SkeletonBone bone=SkeletonBone(0.25, 0.07,RotMat);
		//DHParameter DH = DHParameter(0, 0, 0, 0, std::pair <double,double>(-PI/2.0,(3.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
		//S.mergeSkeletonbyJointVertex(pinky.getSkeleton(), v1, ring.getSkeleton(), v2, DH, bone);
	};

	void Hand::setPinky(Pinky pinky){
		this->pinky=pinky;

	};

	void Hand::setRing(Ring ring){
		this->ring=ring;
	};

	void Hand::setMiddle(Middle middle){
		this->middle=middle;
	};

	void Hand::setIndex(Index index){
		this->index=index;
	};

	void Hand::setThumb(Thumb thumb){
		this->thumb=thumb;
	};

	Hand::~Hand(){
	};

}


