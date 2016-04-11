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

#pragma once
#ifndef ARM_H
#define ARM_H

#include <iostream>
#include <list>
#include <boost/math/constants/constants.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "Leap.h"
//#include "LeapToQ2.h"
#include <Skeleton.h>


namespace Q2{

	//using namespace Q2;

	/** @brief Finger is a parent class of Pinky, Ring, MiddleIndex, Thumb
		@image html C:/Users/foelbern/BeMobil/Code/git%20code/fingernames.png
	*/
	class Arm
	{
	public:
		Arm();
		Arm::Arm(bool isRight);
		~Arm();
		void setJointsByType(SkeletonJoint joint, skeletonJointType type);
		void setSkeleton(Skeleton);
		void setTimestamp(int64_t frametimestamp);
		void setIsLeft(bool);


		void getJointsByType(SkeletonJoint &joint_out, skeletonJointType type);
		Skeleton& getSkeleton();

		void setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		void setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		bool update(SkeletonJoint &joint, int DOF, double phi_t, double score);
		bool update(JointVertex &vertex, int DOF, double phi_t, double score);
		bool update();

		void mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone);		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone

		// set joints
		void setWrist(const SkeletonJoint joint);
		void setElbow(const SkeletonJoint joint);
		void setShoulder(const SkeletonJoint joint);

		// set bones
		void setUpperarm(const SkeletonBone joint);
		void setForearm(const SkeletonBone joint);

		// get joints
		SkeletonJoint getWrist();
		SkeletonJoint getElbow();
		SkeletonJoint getShoulder();

		// get bones
		SkeletonBone getUpperarm();
		SkeletonBone getForearm();

		bool getIsLeft();
		bool getIsRight();

		int64_t timestamp();

	protected:
		Skeleton S;
		JointVertex shoulder, elbow, wrist;
		BoneEdge forearm, upperarm;
		int64_t frametimestamp;
		bool isLeft;
		bool isRight;
	};
}

#endif // end ARM_H