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

#include <iostream>
#include <list>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "Leap.h"
#include "LeapToQ2.h"
#include <Skeleton.h>

namespace Q2
{

	/** @brief Hand model containing hand skeleton which consists of joints and bones
	*/
	class Hand
	{
	public:
		Hand();
		Hand(bool isRight);
		Hand(Pinky pinky, Ring ring, Middle middle, Index index, Thumb thumb);
		~Hand();
		void setJointsByType(SkeletonJoint joint, skeletonJointType type);

		void getJointsByType(SkeletonJoint &joint_out, skeletonJointType type);
		
		Skeleton mergeSkeletonbyJointsType(skeletonJointType type, DHParameter DH, Skeleton& S, SkeletonJoint& joint, SkeletonBone bone);

		void setPinky(Pinky pinky);
		void setRing(Ring ring);
		void setMiddle(Middle middle);
		void setIndex(Index index);
		void setThumb(Thumb thumb);

	private:
		Skeleton S;
		SkeletonJoint CMC;
		bool isLeft;
		bool isRight;
		Pinky pinky;
		Ring ring;
		Middle middle;
		Index index;
		Thumb thumb;
	};

}



