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

#ifndef ARMA5_H
#define ARMA5_H

#include <iostream>
#include <list>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "Leap.h"
//#include "LeapToQ2.h"
#include <Skeleton.h>

namespace Q2{

	/** @brief Arm model for subproject A5_V01
		*/
	class ArmA5
	{
	
	public:
		ArmA5(bool isRight, int32_t ID);
		~ArmA5();
		void setSkeleton(Skeleton);
		void setTimestamp(int64_t frametimestamp);
		void setSoulder(const SkeletonJoint joint);
		void setElbow(const SkeletonJoint joint);
		void setTcp(const SkeletonJoint joint);
		void setJointsByType(SkeletonJoint joint, skeletonJointType type);
		void setIsLeft(bool);
		//void setIsRight1(bool);

		/** @brief get shoulder joint
		*/
		Skeleton getSkeleton();
		bool getShoulder(SkeletonJoint&);
		bool getElbow(SkeletonJoint&);
		bool getWrist(SkeletonJoint&);
		bool getMiddeltip(SkeletonJoint&);
		bool getShoulderJointDOF(JointDOF &joint_out, movementType type);
		bool getElbowJointDOF(JointDOF &joint_out, movementType type);
		bool getWristJointDOF(JointDOF &joint_out, movementType type);
		bool getMiddelhandJointDOF(JointDOF &joint_out, movementType type);
		/** @brief Get forearm bone
			*/
		SkeletonBone getForearm();
		SkeletonBone getUpperarm();
		SkeletonBone getMiddlehand();
		void getJointsByType(SkeletonJoint &joint_out, skeletonJointType type);
		void getJointDOFbyMovementType(SkeletonJoint &joint_out, movementType type);
		bool getIsLeft();
		bool getIsRight();
		int32_t getID();

		/** @brief Set the joint which is beginning of the kinematic chain.
			*  
			*	All joints are recursivly updated based on the start joint
			*/
		void setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		
		/** @brief Set the joint which is beginning of the kinematic chain.
			*  
			*	All joints are recursivly updated based on the start joint
			*	@param Vertex vertex of the joint. Is the number of the joint added to the graph of the skeleton (kinematic chain)
			*  @param PositionGlobal Global position of the strat joint
			*  @param RotMatGlobal Global rotation matrix of the joint
			*/
		void setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		bool update();

		/** @brief Set angle phi to a DOF of a joint
			*/
		bool update(SkeletonJoint &joint, int DOF, double phi_t, double score);

		/** @brief Set angle phi to a DOF of a joint (based on joint vertex)
			*/
		bool update(JointVertex &vertex, int DOF, double phi_t, double score);

		/** @brief get position of the shoulder
			*/
		Eigen::Vector3d shoulderPosition();

		/** @brief get position of the elbow
			*/
		Eigen::Vector3d elbowPosition();

		/** @brief Get position of the wrist
			*/
		Eigen::Vector3d wristPosition();
		/** @brief Get position of the middelhand
			*/
		Eigen::Vector3d middelhandPosition();

		/** @brief Get angle phi of shoulder rotation in the saggital plane
			*/
		double shoulder_FE();
		/** @brief Get angle phi of shoulder rotation in the frontal plane
			*/
		double shoulder_AA();
		/** @brief Get angle phi of shoulder rotation in the transversal plane
			*/
		double shoulder_PS();
		/** @brief Get angle phi of shoulder rotation in the transversal plane
			*/
		double elbow_FE();
		/** @brief Get angle phi of wrist rotation in the saggital plane
			*/
		double wrist_FE();
		/** @brief Get angle phi of wrist rotation in the frontal plane
			*/
		double wrist_AA();
		/** @brief Get angle phi of wrist rotation in the transversal plane
			*/
		double wrist_PS();
		/** @brief Get Frame timestamp
			*/
		int64_t timestamp();


	private:
		Skeleton S;
		JointVertex shoulder, elbow, wrist, middelhand;
		BoneEdge forearm, upperarm, hand;
		bool isLeft;
		bool isRight;
		int64_t frametimestamp;
		int32_t id;

	};

}

#endif // ArmA5_H