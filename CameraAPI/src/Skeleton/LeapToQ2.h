#pragma once
/*
	*	The MIT License (MIT)
	*	
	*	Copyright (c) 2015 Föllmer, Bernhard
	*	
	*	Permission is hereby granted, free of charge, to any person obtaining a copy
	*	of this software and associated documentation files (the "Software"), to deal
	*	in the Software without restriction, including without limitation the rights
	*	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	*	copies of the Software, and to permit persons to whom the Software is
	*	furnished to do so, subject to the following conditions:
	*	
	*	The above copyright notice and this permission notice shall be included in all
	*	copies or substantial portions of the Software.
	*	
	*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	*	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,SkeletonJoint
	*	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	*	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	*	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	*	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	*	SOFTWARE.
*/
#ifndef LEAPTOQ2_H
#define LEAPTOQ2_H

#include "Finger.h"
#include <iostream>
#include <list>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "Leap.h"
#include "Skeleton.h"
#include "ArmA5.h"
#include "Arm.h"
#include "HandArm.h"
#include <iostream>
#include <list>



/** @brief Namespace LeapToQ2
*/
namespace LeapToQ2{


	Eigen::Matrix3d LeapToQ2Rot();
	const Eigen::Matrix3d T4_forearm_right();
	const Eigen::Matrix3d T4_forearm_left();
	const Eigen::Matrix3d T4_hand();
	
	//void LeapRotationHand(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation);
	void LeapRotationArm(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation);

	void LeapRotationBone(Leap::Bone &bone, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation, bool right);
	//void LeapRotationHand(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation);

	bool updateFromLeapArm(Leap::Hand &hand, Q2::Arm &arm);

	bool updateFromLeapHand(Leap::Hand &hand, Q2::ArmA5 &arm);

	bool updateFromLeapHandArm(Leap::Hand &hand, Q2::HandArm &handarm, int64_t timestamp);

	bool updateFromLeapFinger(Leap::Hand &hand, Leap::Finger finger_leap, Q2::Finger &finger);

	bool updateFromLeapMCBone(Q2::HandArm &handarm, Leap::Hand &hand, Leap::Finger, bool right);

	void LeapMatToEigenMat(Leap::Matrix &LeapMat, Eigen::Matrix3d &EigenMat);

	
	/** @brief Transform a 3d point in the leap motion coordinate system to the A5 global coordinate system
	*/
	void LeapToQ2Coord(Eigen::Vector3d &leap_coord, Eigen::Vector3d &q2_coord);

	void EigenMatToLeapMat(Leap::Matrix &LeapMat, Eigen::Matrix3d &EigenMat);
}

#endif

