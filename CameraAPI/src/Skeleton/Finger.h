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
#ifndef FINGER_H
#define FINGER_H

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
	class Finger
	{
	public:
		Finger();
		~Finger();
		void setIsTouched(bool);
		void setJointsByType(SkeletonJoint joint, skeletonJointType type);
		void setSkeleton(Skeleton);

		void getJointsByType(SkeletonJoint &joint_out, skeletonJointType type);
		bool getIsTouched();
		Skeleton& getSkeleton();
		

		void setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		void setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal);
		bool update(SkeletonJoint &joint, int DOF, double phi_t, double score);
		bool update(JointVertex &vertex, int DOF, double phi_t, double score);
		bool update();

		bool isright();

		void mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone);		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone

	protected:
		bool right;
		bool left;
		bool isTouched;
		Skeleton S;
	};

	/** @brief Pinky is the finger pinky
	*/
	class Pinky : public Finger
	{
	public:
		Pinky();
		Pinky(bool isright);
		~Pinky();

		// set joints
		void setTcp(const SkeletonJoint joint);
		void setDip(const SkeletonJoint joint);
		void setPip(const SkeletonJoint joint);
		void setMcp(const SkeletonJoint joint);

		// set bones
		void setDP(const SkeletonBone joint);
		void setMP(const SkeletonBone joint);
		void setPP(const SkeletonBone bone);

		// get joints
		SkeletonJoint getMcp();
		SkeletonJoint getPip();
		SkeletonJoint getDip();
		SkeletonJoint getTip();

		// get bones
		SkeletonBone getMC();
		SkeletonBone getPP();
		SkeletonBone getMP();
		SkeletonBone getDP();

	private:
		JointVertex wrist, mcp, dip, pip, tip;
		BoneEdge mc, pp, mp, dp;
	};

	/** @brief Ring is the finger ring
	*/
	class Ring : public Finger
	{
	public:
		Ring();
		Ring(bool isright);
		~Ring();
		void setTcp(const SkeletonJoint joint);
		void setDip(const SkeletonJoint joint);
		void setPip(const SkeletonJoint joint);
		void setMcp(const SkeletonJoint joint);
		
		void setDP(const SkeletonBone joint);
		void setMP(const SkeletonBone joint);
		void setPP(const SkeletonBone bone);

		SkeletonJoint getDip();
		SkeletonJoint getPip();
		SkeletonJoint getMcp();
		SkeletonJoint getTip();

		SkeletonBone getMC();
		SkeletonBone getPP();
		SkeletonBone getMP();
		SkeletonBone getDP();

	private:
		JointVertex wrist, mcp, dip, pip, tip;
		BoneEdge mc, pp, mp, dp;
	};

	/** @brief Middle is the finger middle
	*/
	class Middle : public Finger
	{
	public:
		Middle();
		Middle(bool isright);
		~Middle();
		void setTcp(const SkeletonJoint joint);
		void setDip(const SkeletonJoint joint);
		void setPip(const SkeletonJoint joint);
		void setMcp(const SkeletonJoint joint);

		void setDP(const SkeletonBone joint);
		void setMP(const SkeletonBone joint);
		void setPP(const SkeletonBone bone);

		SkeletonJoint getDip();
		SkeletonJoint getPip();
		SkeletonJoint getMcp();
		SkeletonJoint getTip();

		SkeletonBone getMC();
		SkeletonBone getPP();
		SkeletonBone getMP();
		SkeletonBone getDP();

	private:
		JointVertex wrist, mcp, dip, pip, tip;
		BoneEdge mc, pp, mp, dp;
	};

	/** @brief Index is the finger index
	*/
	class Index : public Finger
	{
	public:
		Index();
		Index(bool isright);
		~Index();
		void setTcp(const SkeletonJoint joint);
		void setDip(const SkeletonJoint joint);
		void setPip(const SkeletonJoint joint);
		void setMcp(const SkeletonJoint joint);
		
		void setDP(const SkeletonBone joint);
		void setMP(const SkeletonBone joint);
		void setPP(const SkeletonBone bone);

		SkeletonJoint getMcp();
		SkeletonJoint getDip();
		SkeletonJoint getPip();
		SkeletonJoint getTip();

		SkeletonBone getMC();
		SkeletonBone getPP();
		SkeletonBone getMP();
		SkeletonBone getDP();
	private:

		JointVertex wrist, mcp, dip, pip, tip;
		BoneEdge mc, pp, mp, dp;
	};

	/** @brief Thumb is the finger thumb
	*/
	class Thumb : public Finger
	{
	public:
		Thumb();
		Thumb(bool isright);
		~Thumb();
		void setTcp(const SkeletonJoint joint);
		void setIp(const SkeletonJoint joint);
		void setMcp(const SkeletonJoint joint);
		void setCmc(const SkeletonJoint joint);

		void setDP(const SkeletonBone joint);
		void setMP(const SkeletonBone joint);
		void setPP(const SkeletonBone bone);

		
		
		SkeletonJoint getMcp();
		SkeletonJoint getCmc();
		SkeletonJoint getIp();
		SkeletonJoint getTip();

		SkeletonBone getMC();
		SkeletonBone getPP();
		SkeletonBone getMP();
		SkeletonBone getDP();
		
		

	private:
		JointVertex wrist, mcp, cmc, ip, tip;
		BoneEdge mc, dp, mp, pp;
	};
}

#endif // end FINGER_H