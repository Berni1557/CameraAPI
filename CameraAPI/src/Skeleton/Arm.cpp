/**
@file ArmA5.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/

/*
#include "Arm.h"

const double PI = boost::math::constants::pi<double>();

namespace Q2{

	//using namespace Q2;

	Arm::Arm(){
	};

	Arm::Arm(bool isRight){
		
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
			DHParameter DH_shoulder_FE=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0, (1.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_shoulder_PS=DHParameter(PI/2, 0, PI/2, 0, std::pair <double,double>(-PI, (1.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_elbow_FE=DHParameter(PI, upperarm_length, PI/2, 0, std::pair <double,double>(-(1.0/9.0)*PI,PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi


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

			// Create bones
			Eigen::Matrix3d RotMatLocal;
			RotMatLocal=Eigen::Matrix3d::Identity();
			SkeletonBone upperarm_bone=SkeletonBone(upperarm_length, upperarm_width ,RotMatDefaultElbow, RotMatUnionElbow);
			SkeletonBone forearm_bone=SkeletonBone(forearm_length, forearm_width, RotMatDefaultWrist, RotMatUnionWrist);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(shoulder_joint);
			S_graph.addJoint(elbow_joint);
			S_graph.addJoint(wrist_joint);

			// Add bones to graph
			S_graph.addBone(upperarm_bone,shoulder_joint,elbow_joint);
			S_graph.addBone(forearm_bone,elbow_joint,wrist_joint);

		
			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			//std::cout << RotMatGlobal << std::endl;
			S_graph.setStartJoint(elbow_joint, PositionGlobal, RotMatGlobal);
		
			// Set vertices
			shoulder=shoulder_joint.getVertex();
			elbow=elbow_joint.getVertex();
			wrist=wrist_joint.getVertex();

			// Set edges
			forearm=forearm_bone.getEdge();
			upperarm=upperarm_bone.getEdge();

			// Set Graph
			S=S_graph;
			//S.update();
		}else{
			
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

			// Create bones
			Eigen::Matrix3d RotMatLocal;
			RotMatLocal=Eigen::Matrix3d::Identity();
			SkeletonBone upperarm_bone=SkeletonBone(upperarm_length, upperarm_width, RotMatDefaultElbow, RotMatUnionElbow);
			SkeletonBone forearm_bone=SkeletonBone(forearm_length, forearm_width, RotMatDefaultWrist, RotMatUnionWrist);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(shoulder_joint);
			S_graph.addJoint(elbow_joint);
			S_graph.addJoint(wrist_joint);

			// Add bones to graph
			S_graph.addBone(upperarm_bone,shoulder_joint,elbow_joint);
			S_graph.addBone(forearm_bone,elbow_joint,wrist_joint);

		
			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			//std::cout << RotMatGlobal << std::endl;
			S_graph.setStartJoint(elbow_joint, PositionGlobal, RotMatGlobal);
		
			// Set vertices
			shoulder=shoulder_joint.getVertex();
			elbow=elbow_joint.getVertex();
			wrist=wrist_joint.getVertex();

			// Set edges
			forearm=forearm_bone.getEdge();
			upperarm=upperarm_bone.getEdge();

			// Set Graph
			S=S_graph;
			//S.update();
		}
	};

	void Arm::setSkeleton(Skeleton S){
		this->S=S;
	};

	void Arm::setTimestamp(int64_t frametimestamp){
		this->frametimestamp=frametimestamp;
	};

	void Arm::setIsLeft(bool isLeft){
		this->isLeft=isLeft;
		this->isRight=!isLeft;
	};

	Skeleton& Arm::getSkeleton(){
		return this->S;
	};

	void Arm::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(joint, PositionGlobal, RotMatGlobal);
	};

	void Arm::setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(vertex, PositionGlobal, RotMatGlobal);
	};

	bool Arm::update(SkeletonJoint &joint, int DOF, double phi_t, double score){
		return S.update(joint, DOF, phi_t, score);
	};

	bool Arm::update(JointVertex &vertex, int DOF, double phi_t, double score){
		return S.update(vertex, DOF, phi_t, score);
	};
	
	bool Arm::update(){
		return S.update();
	};

	void Arm::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone){		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		this->S.mergeSkeletonbyJointVertex(S1, v1, S2, v2, DH, bone);
	};


	SkeletonJoint Arm::getShoulder(){
		if(S.getMapping()){
			JointVertex v=S.getMap()[shoulder];
			SkeletonJoint s=S.getMappingSkeleton()->getGraph()[v];
			return S.getMappingSkeleton()->getGraph()[v];
		}else{
			return S.getGraph()[shoulder];
		}
	};

	SkeletonJoint Arm::getElbow(){
		if(S.getMapping()){
			JointVertex v=S.getMap()[elbow];
			SkeletonJoint s=S.getMappingSkeleton()->getGraph()[v];
			return S.getMappingSkeleton()->getGraph()[v];
		}else{
			return S.getGraph()[elbow];
		}
	};

	SkeletonJoint Arm::getWrist(){
		if(S.getMapping()){
			JointVertex v=S.getMap()[wrist];
			SkeletonJoint s=S.getMappingSkeleton()->getGraph()[v];
			return S.getMappingSkeleton()->getGraph()[v];
		}else{
			return S.getGraph()[wrist];
		}
	};


	SkeletonBone Arm::getUpperarm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(shoulder, g);
		return S.getGraph()[*ei];
	};

	SkeletonBone Arm::getForearm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(elbow, g);
		return S.getGraph()[*ei];
	};

	bool Arm::getIsLeft(){
		return isLeft;
	};

	bool Arm::getIsRight(){
		return isRight;
	};


	int64_t Arm::timestamp(){
		return frametimestamp;
	};

	Arm::~Arm(){
	};

}


*/