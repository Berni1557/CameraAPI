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
#ifndef SKELETON_H
#define SKELETON_H


#include <iostream>
#include <list>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
//#include "LeapToQ2.h"
#include <windows.h> 
#include "Sphere.h"
#include "Cylinder.h"

//#include "GLWidget.h" 

namespace Q2
{
	class SkeletonJoint;
	class SkeletonBone;

	typedef boost::adjacency_list<boost::vecS,boost::vecS,boost::bidirectionalS , SkeletonJoint, SkeletonBone> SkeletonGraph;
	typedef boost::graph_traits<SkeletonGraph>::vertex_descriptor JointVertex;
	typedef boost::graph_traits<SkeletonGraph>::vertex_iterator vertex_iter;

	typedef boost::graph_traits<SkeletonGraph>::edge_descriptor BoneEdge;	
	typedef boost::graph_traits<SkeletonGraph>::edge_iterator edge_iter;

	typedef boost::graph_traits<SkeletonGraph>::out_edge_iterator out_edge_iter;
	typedef boost::graph_traits<SkeletonGraph>::in_edge_iterator in_edge_iter;

	typedef boost::graph_traits<SkeletonGraph>::adjacency_iterator adjacency_it;

	typedef std::map<JointVertex, JointVertex> vertex_map;

	/** @brief Transformation from Leap-upper arm coordinate system to Default-upper arm coordinate system
	*/
	//const Eigen::Matrix3d T4_forearm=Eigen::Matrix3d::Zero(); 

	//T4_forearm(0,2)=-1;
	//T4_forearm(1,1)=-1;
	//T4_forearm(2,0)=-1;

	/** @brief Transform a rotation matrix in the leap motion coordinat system to the A5 global coordinat system
	*/	
	//void LeapToQ2Mat(Eigen::Matrix3d &leap_Rotmat, Eigen::Matrix3d &q2_Rotmat);






	bool EulerFromMat(Eigen::Matrix3d &R, double &alpha, double alpha_max,  double alpha_min, double &beta, double beta_max,  double beta_min, double &gamma, double gamma_max,  double gamma_min);

	Eigen::Matrix3d LeapToQ2Rot();

	enum edgeDirection{
		Edge_IN = 0,
		Edge_OUT = 1
	};

	enum fingerType{
		PINKY = 0,
		RING = 1, 
		MIDDLE = 2, 
		INDEX = 3, 
		THUMB = 4
	};

	enum skeletonBoneType{
		DP = 0,
		MP = 1,
		PP = 2,
		MC  = 3
	};

	enum skeletonJointType{
		CMC = 0,
		MCP = 1,
		DIP = 2,
		PIP  = 3,
		IP  = 4,
		TIP = 5
	};

	enum movementType{
		FE = 0,
		AA = 1,
		PS = 2,
		NM = 3, // No movement
	};

	// shape type
	enum shapeType{
		sphere = 0,
		cylinder = 1,
		cone = 2,
	};

	Eigen::Matrix3d euler( const double roll,const double pitch,const double yaw );
	/*
	class Hand
	{
	public:
		Hand();
		~Hand();	
		void setFinger(const SkeletonJoint, const SkeletonJoint, const SkeletonJoint, const SkeletonJoint, fingerType);
		void setFinger(const SkeletonJoint, const SkeletonJoint, const SkeletonJoint);
		void getFinger(Finger &, fingerType);

		Finger pinky, ring, middle, index, thumb;
	};
	*/

	/*
	class Arm : Skeleton
	{
	public:
		Arm();
		~Arm();
		void setIsLeft(const bool isLeft); 
		void setIsRight(const bool isRight);
		void setIsLeft(); 
		void setIsRight(); 
		void setHand();
		fingerType currentFinger;

	private:
		bool isLeft;
		bool isRight;
		//Hand hand;
		SkeletonJoint startingJoint;
		SkeletonJoint shoulder;
		SkeletonJoint ellbow;
	};

	*/

	/*! \brief Denavit–Hartenberg parameters for each DOF of a joint
	 *
	 */
	class DHParameter
	{
	public:
		DHParameter(double phi, double d, double alpha, double a, std::pair <double,double> MinMaxPhi);
		DHParameter();
		~DHParameter();
		void setPhi(double);
		bool setPhi_t(double);
		void setMinMaxPhi(std::pair <double,double>);
		void setD(double);
		void setAlpha(double);
		void setA(double);

		double DHParameter::getPhi();
		double DHParameter::getPhi_t();
		double DHParameter::getD();
		Eigen::Matrix4d DHParameter::getRotz();
		Eigen::Matrix4d DHParameter::getRotz(double);
		Eigen::Matrix4d DHParameter::getTransz();
		Eigen::Matrix4d DHParameter::getTransz(double);
		Eigen::Matrix4d DHParameter::getRotx();
		Eigen::Matrix4d DHParameter::getTransx();
		Eigen::Matrix4d DHParameter::getT();
		std::pair <double,double> DHParameter::getMinMaxPhi();

	private:
		double phi;
		double phi_t;
		std::pair <double,double> MinMaxPhi;
		double d;
		double alpha;
		double a;
		Eigen::Matrix<double,4,4,Eigen::DontAlign> T;
	};

	/*! \brief DOF of a joint
	 *	Each joint has between one and three DOFs
	 */
	class JointDOF
	{
	public:
		JointDOF();
		JointDOF(DHParameter,movementType);
		~JointDOF();
		void setDHParameter(DHParameter);
		void setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setPositionGlobal(Eigen::Vector3d);
		void setQuaternionGlobal(Eigen::Quaternion<double,Eigen::DontAlign>);
		void setMtype(movementType);
		void setScore(double);

		DHParameter& getDHParameter();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatGlobal();
		Eigen::Vector3d getPositionGlobal();
		Eigen::Quaternion<double,Eigen::DontAlign> getQuaternionGlobal();
		movementType getMtype();
		double getScore();

		bool updatePhi_t(double phi_t);
		

	private:
		DHParameter DH;
		movementType mtype;
		Eigen::Vector3d PositionGlobal;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal;
		Eigen::Quaternion<double,Eigen::DontAlign> q;
		double score;
	};

	/** @brief SkeletonJoint represents a joint in the skeleton
	*/
	class SkeletonJoint
	{
	public:
		//@brief initialises orientation and boneLength to 0 and isValid to false
		SkeletonJoint();
		SkeletonJoint(std::vector <JointDOF>,skeletonJointType type, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion);
		~SkeletonJoint();
		void setDOFvec(std::vector <JointDOF>);
		void setType(skeletonJointType);
		void setVertex(JointVertex);
		void setPositionGlobal(Eigen::Vector3d PositionGlobal);
		void setQGlobal(Eigen::Quaternion<double,Eigen::DontAlign>);
		void setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setRotMatDefault(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setRotMatDefaultToUnion(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setShape(Sphere);

		std::vector <JointDOF>& getDOFvec();
		Eigen::Vector3d getPositionGlobal();
		Eigen::Quaternion<double,Eigen::DontAlign> getQGlobal();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatGlobal();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatDefault();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatDefaultToUnion();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatUnion();
		Eigen::Quaternion<double,Eigen::DontAlign> getQuaternionUnion();
		Sphere getShape();
		
		Eigen::Quaternion<double,Eigen::DontAlign> q;
		bool updatePhi(int DOF, double phi_t, double score);
		bool update(SkeletonGraph &);
		bool update(SkeletonGraph &, SkeletonJoint &, edgeDirection);
		JointVertex getVertex();
		bool update_in_joints(SkeletonGraph &);
		bool update_out_joints(SkeletonGraph &);
		bool update_in_joints(SkeletonGraph &, SkeletonJoint &);
		bool update_out_joints(SkeletonGraph &, SkeletonJoint &);
		
		bool update_Position_Orientation(SkeletonJoint &origen, edgeDirection edgeDir);
		bool update_Position_Orientation();

		bool getJointDOFbyMovementType(JointDOF &, movementType type);

		bool init(shapeType Stype);


	private:
		std::vector <JointDOF> DOFvec;
		skeletonJointType type;
		Eigen::Vector3d PositionGlobal;
		Eigen::Quaternion<double,Eigen::DontAlign> qGlobal;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion;
		JointVertex vertex;
		Sphere JointShape;
	};

	/** @brief SkeletonBone represents a bone in the skeleton
	*/
	class SkeletonBone
	{
	public:
		SkeletonBone();
		SkeletonBone(double boneLength,Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion);
		SkeletonBone(double boneLength, double boneWidth, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion);
		~SkeletonBone();

		void setEdge(BoneEdge);
		void setPositionGlobal(Eigen::Vector3d PositionGlobal);
		void setQGlobal(Eigen::Quaternion<double,Eigen::DontAlign>);
		void setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setRotMatDefault(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setRotMatDefaultToUnion(Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setBoneLength(double);
		void setShape(Cylinder BoneShape);

		BoneEdge getEdge();
		Eigen::Vector3d getPositionGlobal();
		Eigen::Quaternion<double,Eigen::DontAlign> getQGlobal();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatGlobal();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatDefault();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatDefaultToUnion();
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatUnion();
		Eigen::Quaternion<double,Eigen::DontAlign> getQuaternionUnion();
		double getBoneLength();
		Cylinder getShape();

		bool update(SkeletonJoint&);
		bool init(shapeType Stype);

	private:
		double boneLength;
		double boneWidth;
		BoneEdge edge;
		Eigen::Vector3d PositionGlobal;
		Eigen::Quaternion<double,Eigen::DontAlign> qGlobal;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault;
		Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion;
		Cylinder BoneShape;
		
	};

	/** @brief Skeleton consists of joints which are connectes with bones
	*/
	class Skeleton
	{
	public:
		Skeleton();
		~Skeleton();
		void show();
		bool addJoint(SkeletonJoint &joint);
		bool addJoint(SkeletonJoint &joint, JointVertex &joint_vertex, SkeletonBone bone, JointVertex &joint_vertex_out);
		bool addBone(SkeletonBone bone, SkeletonJoint &joint_start, SkeletonJoint &joint_end);
		void addSkeleton(JointVertex joint_skeleton, Skeleton skeleton, JointVertex joint);
		
		Eigen::Vector3d getGlobalPositionVertex(JointVertex joint);
		Eigen::Quaternion<double,Eigen::DontAlign> getQGlobalVertex(JointVertex joint);
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatGlobalVertex(JointVertex joint);

		Eigen::Vector3d getGlobalPositionEdge(JointVertex jv_start, JointVertex jv_end);
		Eigen::Quaternion<double,Eigen::DontAlign> getQGlobalEdge(JointVertex jv_start, JointVertex jv_end);
		Eigen::Matrix<double,3,3,Eigen::DontAlign> getRotMatGlobalEdge(JointVertex jv_start, JointVertex jv_end);

		vertex_map getMap();
		bool getMapping();
		
		
		void setStartJoint(SkeletonJoint &joint,  Eigen::Vector3d PositionGlobal, Eigen::Quaternion<double,Eigen::DontAlign>);
		void setStartJoint(SkeletonJoint &joint,  Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setStartJoint(JointVertex &vertex,  Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign>);
		void setStartJoint(JointVertex &vertex);
		void setGraph(SkeletonGraph);
		void setMap(vertex_map map);
		void setMapping(bool mapping);
		void setMappingSkeleton(Skeleton& MappingSkeleton);

		SkeletonJoint* getStartJoint();
		Skeleton* getMappingSkeleton();

		bool update();
		bool update(SkeletonJoint &joint, int DOF, double phi_t,double score);
		bool update(JointVertex vertex, int DOF, double phi_t, double score);

		Eigen::VectorXd getState();
		Eigen::VectorXd Skeleton::getMinState();
		Eigen::VectorXd Skeleton::getMaxState();

		SkeletonGraph& getGraph();

		bool mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone);		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		bool mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2);		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		bool mergeSkeletonbyJointVertex(JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone);		//  source vertex, target skeleton, target vertex, DH parameter, Bone

		bool init(shapeType JointShapeType, shapeType BoneShapeType);
		bool draw();

	private:
		SkeletonGraph Graph;
		SkeletonJoint* StartJoint;
		vertex_map VMap;
		Skeleton* MappingSkeleton;
		bool Mapping;
		
	};


	/*
	class ArmHand
	{
	public:
		ArmHand();
		~ArmHand();
		void setArm(const Arm arm);
		void setHand(const Hand hand);

	private:
		Skeleton S;
		Arm arm;
		Hand hand;
	};
	*/
}

#endif // end SKELETON_H

