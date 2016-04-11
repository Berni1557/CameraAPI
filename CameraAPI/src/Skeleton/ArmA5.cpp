/**
@file ArmA5.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/
#include "ArmA5.h"
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


const double PI = boost::math::constants::pi<double>();

namespace Q2{

	ArmA5::ArmA5(bool isRight, int32_t ID){

		this->id=ID;
		double hand_length=0.18;
		double hand_width=0.03;
		double upperarm_length=0.25;
		double upperarm_width=0.07;
		double forearm_length=0.3;
		double forearm_width=0.05;

		if(isRight){
			// Left Arm
			this->isRight=isRight;
			this->isLeft=!isRight;

			// Create DH parameter
			DHParameter DH_shoulder_AA=DHParameter(0, 0, 0, 0, std::pair <double,double>(-PI/2.0,(3.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_shoulder_FE=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0, 0.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_shoulder_PS=DHParameter(PI/2, 0, PI/2, 0, std::pair <double,double>(-PI, 0.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_elbow_FE=DHParameter(PI, upperarm_length, PI/2, 0, std::pair <double,double>(0.0,PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_middelhand_TIP=Q2::DHParameter(PI/2, -hand_length, PI/2, 0, std::pair <double,double>(0.f,0.f));// phi, d, alpha, a, MinMaxPhi


			// Create joints

			// Create shoulder joint
			std::vector <JointDOF> shoulderDOF;
			shoulderDOF.push_back(JointDOF(DH_shoulder_AA,AA));
			shoulderDOF.push_back(JointDOF(DH_shoulder_FE,FE));
			shoulderDOF.push_back(JointDOF(DH_shoulder_PS,PS));
			Eigen::Matrix3d RotMatDefaultShoulder=Eigen::Matrix3d::Zero();
			RotMatDefaultShoulder(0,2)=1;
			RotMatDefaultShoulder(1,1)=1;
			RotMatDefaultShoulder(2,0)=-1;
			Eigen::Matrix3d RotMatUnionShoulder=Eigen::Matrix3d::Zero();
			RotMatUnionShoulder(0,0)=1;
			RotMatUnionShoulder(1,1)=1;
			RotMatUnionShoulder(2,2)=1;
			SkeletonJoint shoulder_joint=SkeletonJoint(shoulderDOF, CMC, RotMatDefaultShoulder, RotMatUnionShoulder);		

			// Create elbow joint
			std::vector <JointDOF> elbowDOF;
			elbowDOF.push_back(JointDOF(DH_elbow_FE,FE));
			Eigen::Matrix3d RotMatDefaultElbow=Eigen::Matrix3d::Zero();
			RotMatDefaultElbow(0,0)=-1;
			RotMatDefaultElbow(1,2)=-1;
			RotMatDefaultElbow(2,1)=-1;
			Eigen::Matrix3d RotMatUnionElbow=Eigen::Matrix3d::Zero();
			RotMatUnionElbow(0,1)=1;
			RotMatUnionElbow(1,2)=-1;
			RotMatUnionElbow(2,0)=-1;
			SkeletonJoint elbow_joint=SkeletonJoint(elbowDOF,CMC,RotMatDefaultElbow, RotMatUnionElbow);

			// Create wrist joint
			std::vector <JointDOF> wristDOF;
			wristDOF.push_back(JointDOF(DH_wrist_FE,FE));
			wristDOF.push_back(JointDOF(DH_wrist_AA,AA));
			wristDOF.push_back(JointDOF(DH_wrist_PS,PS));
			Eigen::Matrix3d RotMatDefaultWrist=Eigen::Matrix3d::Zero();
			RotMatDefaultWrist(0,1)=1;
			RotMatDefaultWrist(1,2)=-1;
			RotMatDefaultWrist(2,0)=-1;
			Eigen::Matrix3d RotMatUnionWrist=Eigen::Matrix3d::Zero();
			RotMatUnionWrist(0,0)=1;
			RotMatUnionWrist(1,2)=1;
			RotMatUnionWrist(2,1)=-1;
			SkeletonJoint wrist_joint=SkeletonJoint(wristDOF,TIP,RotMatDefaultWrist, RotMatUnionWrist);

			// Create middelhand joint
			Eigen::Matrix3d RotMatDefaultMiddelhand=Eigen::Matrix3d::Zero();
			RotMatDefaultMiddelhand(0,0)=-1;
			RotMatDefaultMiddelhand(1,2)=1;
			RotMatDefaultMiddelhand(2,1)=1;
			Eigen::Matrix3d RotMatUnionMiddelhand=Eigen::Matrix3d::Zero();
			RotMatUnionMiddelhand(0,1)=-1;
			RotMatUnionMiddelhand(1,2)=-1;
			RotMatUnionMiddelhand(2,0)=1;
			std::vector <JointDOF> middelhandDOF;
			middelhandDOF.push_back(JointDOF(DH_middelhand_TIP,NM));
			SkeletonJoint middelhand_joint=SkeletonJoint(middelhandDOF,TIP,RotMatDefaultMiddelhand, RotMatUnionMiddelhand);

			// Create bones
			Eigen::Matrix3d RotMatLocal;
			RotMatLocal=Eigen::Matrix3d::Identity();
			SkeletonBone upperarm_bone=SkeletonBone(upperarm_length, upperarm_width ,RotMatDefaultElbow, RotMatUnionElbow);
			SkeletonBone forearm_bone=SkeletonBone(forearm_length, forearm_width, RotMatDefaultWrist, RotMatUnionWrist);
			SkeletonBone hand_bone=SkeletonBone(hand_length,hand_width, RotMatDefaultMiddelhand, RotMatUnionMiddelhand);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(shoulder_joint);
			S_graph.addJoint(elbow_joint);
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(middelhand_joint);

			// Add bones to graph
			S_graph.addBone(upperarm_bone,shoulder_joint,elbow_joint);
			S_graph.addBone(forearm_bone,elbow_joint,wrist_joint);
			S_graph.addBone(hand_bone,wrist_joint,middelhand_joint);

		
			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			//std::cout << RotMatGlobal << std::endl;
			S_graph.setStartJoint(elbow_joint, PositionGlobal, RotMatGlobal);
		
			// Set vertices
			shoulder=shoulder_joint.getVertex();
			elbow=elbow_joint.getVertex();
			wrist=wrist_joint.getVertex();
			middelhand=middelhand_joint.getVertex();

			// Set edges
			forearm=forearm_bone.getEdge();
			upperarm=upperarm_bone.getEdge();
			hand=hand_bone.getEdge();

			// Set Graph
			S=S_graph;
			//S.update();
		}else{
			
			// Left Arm

			this->isRight=isRight;
			this->isLeft=!isRight;

			// Create DH parameter
			DHParameter DH_shoulder_AA=DHParameter(0, 0, 0, 0, std::pair <double,double>(-PI/2.0,(3.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_shoulder_FE=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0, (1.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_shoulder_PS=DHParameter(PI/2, 0, PI/2, 0, std::pair <double,double>(-PI, (1.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_elbow_FE=DHParameter(PI, upperarm_length, PI/2, 0, std::pair <double,double>(-(1.0/9.0)*PI,PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_middelhand_TIP=Q2::DHParameter(PI/2, -hand_length, PI/2, 0, std::pair <double,double>(0.f,0.f));// phi, d, alpha, a, MinMaxPhi


			// Create joints

			// Create shoulder joint
			std::vector <JointDOF> shoulderDOF;
			shoulderDOF.push_back(JointDOF(DH_shoulder_AA,AA));
			shoulderDOF.push_back(JointDOF(DH_shoulder_FE,FE));
			shoulderDOF.push_back(JointDOF(DH_shoulder_PS,PS));
			Eigen::Matrix3d RotMatDefaultShoulder=Eigen::Matrix3d::Zero();
			RotMatDefaultShoulder(0,2)=-1;
			RotMatDefaultShoulder(1,1)=-1;
			RotMatDefaultShoulder(2,0)=-1;
			Eigen::Matrix3d RotMatUnionShoulder=Eigen::Matrix3d::Zero();
			RotMatUnionShoulder(0,0)=1;
			RotMatUnionShoulder(1,1)=-1;
			RotMatUnionShoulder(2,2)=-1;
			SkeletonJoint shoulder_joint=SkeletonJoint(shoulderDOF, CMC, RotMatDefaultShoulder, RotMatUnionShoulder);		

			// Create elbow joint
			std::vector <JointDOF> elbowDOF;
			elbowDOF.push_back(JointDOF(DH_elbow_FE,FE));
			Eigen::Matrix3d RotMatDefaultElbow=Eigen::Matrix3d::Zero();
			RotMatDefaultElbow(0,0)=-1;
			RotMatDefaultElbow(1,1)=1;
			RotMatDefaultElbow(2,2)=-1;
			Eigen::Matrix3d RotMatUnionElbow=Eigen::Matrix3d::Zero();
			RotMatUnionElbow(0,1)=1;
			RotMatUnionElbow(1,2)=1;
			RotMatUnionElbow(2,0)=1;
			SkeletonJoint elbow_joint=SkeletonJoint(elbowDOF,CMC,RotMatDefaultElbow, RotMatUnionElbow);

			// Create wrist joint
			std::vector <JointDOF> wristDOF;
			wristDOF.push_back(JointDOF(DH_wrist_FE,FE));
			wristDOF.push_back(JointDOF(DH_wrist_AA,AA));
			wristDOF.push_back(JointDOF(DH_wrist_PS,PS));
			Eigen::Matrix3d RotMatDefaultWrist=Eigen::Matrix3d::Zero();
			RotMatDefaultWrist(0,1)=-1;
			RotMatDefaultWrist(1,2)=1;
			RotMatDefaultWrist(2,0)=-1;
			Eigen::Matrix3d RotMatUnionWrist=Eigen::Matrix3d::Zero();
			RotMatUnionWrist(0,0)=1;
			RotMatUnionWrist(1,2)=-1;
			RotMatUnionWrist(2,1)=1;
			SkeletonJoint wrist_joint=SkeletonJoint(wristDOF,TIP,RotMatDefaultWrist, RotMatUnionWrist);

			// Create middelhand joint
			Eigen::Matrix3d RotMatDefaultMiddelhand=Eigen::Matrix3d::Zero();
			RotMatDefaultMiddelhand(0,0)=1;
			RotMatDefaultMiddelhand(1,2)=-1;
			RotMatDefaultMiddelhand(2,1)=1;
			Eigen::Matrix3d RotMatUnionMiddelhand=Eigen::Matrix3d::Zero();
			RotMatUnionMiddelhand(0,1)=-1;
			RotMatUnionMiddelhand(1,2)=1;
			RotMatUnionMiddelhand(2,0)=-1;
			std::vector <JointDOF> middelhandDOF;
			middelhandDOF.push_back(JointDOF(DH_middelhand_TIP,NM));
			SkeletonJoint middelhand_joint=SkeletonJoint(middelhandDOF,TIP,RotMatDefaultMiddelhand, RotMatUnionMiddelhand);

			// Create bones
			Eigen::Matrix3d RotMatLocal;
			RotMatLocal=Eigen::Matrix3d::Identity();
			SkeletonBone upperarm_bone=SkeletonBone(upperarm_length, upperarm_width, RotMatDefaultElbow, RotMatUnionElbow);
			SkeletonBone forearm_bone=SkeletonBone(forearm_length, forearm_width, RotMatDefaultWrist, RotMatUnionWrist);
			SkeletonBone hand_bone=SkeletonBone(hand_length, hand_width, RotMatDefaultMiddelhand, RotMatUnionMiddelhand);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(shoulder_joint);
			S_graph.addJoint(elbow_joint);
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(middelhand_joint);

			// Add bones to graph
			S_graph.addBone(upperarm_bone,shoulder_joint,elbow_joint);
			S_graph.addBone(forearm_bone,elbow_joint,wrist_joint);
			S_graph.addBone(hand_bone,wrist_joint,middelhand_joint);

		
			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			//std::cout << RotMatGlobal << std::endl;
			S_graph.setStartJoint(elbow_joint, PositionGlobal, RotMatGlobal);
		
			// Set vertices
			shoulder=shoulder_joint.getVertex();
			elbow=elbow_joint.getVertex();
			wrist=wrist_joint.getVertex();
			middelhand=middelhand_joint.getVertex();

			// Set edges
			forearm=forearm_bone.getEdge();
			upperarm=upperarm_bone.getEdge();
			hand=hand_bone.getEdge();

			// Set Graph
			S=S_graph;
			//S.update();
		}	
	};
	
	void ArmA5::setIsLeft(bool isLeft){
		this->isLeft=isLeft;
		this->isRight=!isLeft;
	};
	/*
	void ArmA5::setIsRight1(bool isRight_in);
		this->isRight=isRight_in;
		this->isLeft=!isRight_in;
	};
	*/

	Skeleton ArmA5::getSkeleton(){
		return S;
	};

	bool ArmA5::getShoulder(SkeletonJoint &joint){
		joint=S.getGraph()[shoulder];
		return true;
	};

	bool ArmA5::getElbow(SkeletonJoint &joint){
		joint=S.getGraph()[elbow];
		return true;
	};

	bool ArmA5::getWrist(SkeletonJoint &joint){
		joint=S.getGraph()[wrist];
		return true;
	};

	bool ArmA5::getMiddeltip(SkeletonJoint &joint){
		joint=S.getGraph()[middelhand];
		return true;
	};

	bool ArmA5::getShoulderJointDOF(JointDOF &joint_out, movementType type){
		return S.getGraph()[shoulder].getJointDOFbyMovementType(joint_out,type);
	};
	
	bool ArmA5::getElbowJointDOF(JointDOF &joint_out, movementType type){
		return S.getGraph()[elbow].getJointDOFbyMovementType(joint_out,type);
	};
	

	bool ArmA5::getWristJointDOF(JointDOF &joint_out, movementType type){
		return S.getGraph()[wrist].getJointDOFbyMovementType(joint_out,type);
	};

	bool ArmA5::getMiddelhandJointDOF(JointDOF &joint_out, movementType type){
		return S.getGraph()[middelhand].getJointDOFbyMovementType(joint_out,type);
	};
		
	SkeletonBone ArmA5::getUpperarm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(shoulder, g);
		return S.getGraph()[*ei];
	};

	SkeletonBone ArmA5::getForearm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(elbow, g);
		return S.getGraph()[*ei];
	};


	SkeletonBone ArmA5::getMiddlehand(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(wrist, g);
		return S.getGraph()[*ei];
	};

	bool ArmA5::getIsLeft(){
		return isLeft;
	};

	bool ArmA5::getIsRight(){
		return isRight;
	};

	int32_t ArmA5::getID(){
		return id;
	};

	void ArmA5::getJointDOFbyMovementType(SkeletonJoint &joint_out, movementType type){

	};

	void ArmA5::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(joint, PositionGlobal, RotMatGlobal);
	};

	void ArmA5::setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(vertex, PositionGlobal, RotMatGlobal);
	};

	bool ArmA5::update(){
		return S.update();
	};

	void ArmA5::setSkeleton(Skeleton skeleton){
		this->S=skeleton;
	};

	void ArmA5::setTimestamp(int64_t frametimestamp){
		this->frametimestamp=frametimestamp;
	};

	bool ArmA5::update(SkeletonJoint &joint, int DOF, double phi_t, double score){
		return S.update(joint, DOF, phi_t, score);
	};

	bool ArmA5::update(JointVertex &vertex, int DOF, double phi_t, double score){
		return S.update(vertex, DOF, phi_t, score);
	};
	
	Eigen::Vector3d ArmA5::shoulderPosition(){
		SkeletonJoint joint;
		this->getShoulder(joint);
		return joint.getPositionGlobal();
	};

	Eigen::Vector3d ArmA5::elbowPosition(){
		SkeletonJoint joint;
		this->getElbow(joint);
		return joint.getPositionGlobal();
	};

	Eigen::Vector3d ArmA5::wristPosition(){
		SkeletonJoint joint;
		this->getWrist(joint);
		return joint.getPositionGlobal();
	};

	/*!
	* \begin{equation} x=2 \end{equation}
	*/
	Eigen::Vector3d ArmA5::middelhandPosition(){
		SkeletonJoint joint;
		this->getMiddeltip(joint);
		return joint.getPositionGlobal();
	};

	/*! The red coordinate system shows rotation in sagittal plane.
     *  \image html "C:\Users\foelbern\BeMobil\Code\gitCode\git\Interface A5\Documentation\images\shoulder_FE.png"
	 *  $
	 *	\begin{matrix}
	 *	a_1 & a_2 & a_3 & a_4 \\
	 *	b_1 & b_2 & b_3 & b_4 \\
	 *	c_1 & c_2 & c_3 & c_4 \\
	 *	d_1 & d_2 & d_3 & d_4
	 *	\end{matrix}
	 *	$  
     */
	double ArmA5::shoulder_FE(){
		JointDOF JDOF;
		this->getShoulderJointDOF(JDOF,FE);
		return JDOF.getDHParameter().getPhi_t();
	};

	/*!
	 *  $
	 *	\begin{matrix}
	 *	a_1 & a_2 & a_3 & a_4 \\
	 *	b_1 & b_2 & b_3 & b_4 \\
	 *	c_1 & c_2 & c_3 & c_4 \\
	 *	d_1 & d_2 & d_3 & d_4
	 *	\end{matrix}
	 *	$  
     */
	double ArmA5::shoulder_AA(){
		JointDOF JDOF;
		this->getShoulderJointDOF(JDOF,AA);
		return JDOF.getDHParameter().getPhi_t();
	};

	double ArmA5::shoulder_PS(){
		JointDOF JDOF;
		this->getShoulderJointDOF(JDOF,PS);
		return JDOF.getDHParameter().getPhi_t();
	};

	double ArmA5::elbow_FE(){
		JointDOF JDOF;
		this->getElbowJointDOF(JDOF,FE);
		return JDOF.getDHParameter().getPhi_t();
	};

	double ArmA5::wrist_FE(){
		JointDOF JDOF;
		this->getWristJointDOF(JDOF,FE);
		return JDOF.getDHParameter().getPhi_t();
	};

	double ArmA5::wrist_AA(){
		JointDOF JDOF;
		this->getWristJointDOF(JDOF,AA);
		return JDOF.getDHParameter().getPhi_t();
	};

	double ArmA5::wrist_PS(){
		JointDOF JDOF;
		this->getWristJointDOF(JDOF,PS);
		return JDOF.getDHParameter().getPhi_t();
	};

	int64_t ArmA5::timestamp(){
		return frametimestamp;
	};

	ArmA5::~ArmA5(){
	};
}
	