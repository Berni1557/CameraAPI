/**
@file ArmA5.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/


#include "Finger.h"
//#include <iostream>
//#include <list>
//#include <boost/math/constants/constants.hpp>
//#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/copy.hpp>
//#include <Eigen/Dense>
//#include <Eigen/Geometry> 
//#include "Leap.h"
//#include "LeapToQ2.h"
//#include <Skeleton.h>
//#include "ArmA5.h"


const double PI = boost::math::constants::pi<double>();

namespace Q2{

	//using namespace Q2;

	Finger::Finger(){
	};

	void Finger::setSkeleton(Skeleton S){
		this->S=S;
	};


	Skeleton& Finger::getSkeleton(){
		return this->S;
	};

	void Finger::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(joint, PositionGlobal, RotMatGlobal);
	};

	void Finger::setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(vertex, PositionGlobal, RotMatGlobal);
	};

	bool Finger::update(SkeletonJoint &joint, int DOF, double phi_t, double score){
		return S.update(joint, DOF, phi_t, score);
	};

	bool Finger::update(JointVertex &vertex, int DOF, double phi_t, double score){
		return S.update(vertex, DOF, phi_t, score);
	};
	
	bool Finger::update(){
		return S.update();
	};

	bool Finger::isright(){
		return this->right;
	};

	void Finger::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone){		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		this->S.mergeSkeletonbyJointVertex(S1, v1, S2, v2, DH, bone);
	};


	Finger::~Finger(){
	
	};

	Pinky::Pinky(){
	};

	Pinky::Pinky(bool right){

		this->right=right;
		this->left=!right;

		double forearm_length=0.3;
		double boneLength_MC=sqrt(0.063*0.063 + 0.040*0.040);

		if(right){
			// Create DH parameter

			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.063, PI/2, -0.040 , std::pair <double,double>(-PI/4.0, PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0 ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.024, std::pair <double,double>(-3.0*PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.015, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

			// Create Wrist joint
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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=-1;
			RotMatDefaultMCP(1,0)=-1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=-1;
			RotMatDefaultPIP(1,2)=1;
			RotMatDefaultPIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=-1;
			RotMatDefaultDIP(1,2)=1;
			RotMatDefaultDIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=-1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);


			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.024, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.020, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.015, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC,wrist_joint,MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;

		}else{

			// Create DH parameter
			DHParameter DH_wrist_FE=DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.063, PI/2, -0.040 , std::pair <double,double>(-PI/9.0, PI/3.0));				// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.024, std::pair <double,double>(-PI/9.0, 3.0*PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.015, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=1;
			RotMatDefaultMCP(1,0)=1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=1;
			RotMatDefaultPIP(1,2)=-1;
			RotMatDefaultPIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=1;
			RotMatDefaultDIP(1,2)=-1;
			RotMatDefaultDIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.024, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.020, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.015, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC, wrist_joint, MCP_joint);
			S_graph.addBone(PP, MCP_joint, PIP_joint);
			S_graph.addBone(MP, PIP_joint, DIP_joint);
			S_graph.addBone(DP, DIP_joint, TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set 
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;
		}

	};

	SkeletonJoint Pinky::getDip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[dip]];
		}else{
			return S.getGraph()[dip];
		}
	};

	SkeletonJoint Pinky::getPip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[pip]];
		}else{
			return S.getGraph()[pip];
		}
	};

	SkeletonJoint Pinky::getMcp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[mcp]];
		}else{
			return S.getGraph()[mcp];
		}
		
	};

	SkeletonJoint Pinky::getTip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[tip]];
		}else{
			return S.getGraph()[tip];
		}
	};

	SkeletonBone Pinky::getMC(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			SkeletonBone b=S.getGraph()[*ei];
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Pinky::getPP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[pip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(pip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Pinky::getMP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[dip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(dip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Pinky::getDP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[tip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(tip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	Pinky::~Pinky(){
	};

	Ring::Ring(){
	};

	Ring::Ring(bool right){

		this->right=right;
		this->left=!right;

		double forearm_length=0.3;
		double boneLength_MC=sqrt(0.065*0.065 + 0.020*0.020);

		if(right){
			// Create DH parameter
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.065, PI/2, -0.020 , std::pair <double,double>(-PI/4.0, PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.033, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.025, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.016, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

			// Create Wrist joint
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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=-1;
			RotMatDefaultMCP(1,0)=-1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=-1;
			RotMatDefaultPIP(1,2)=1;
			RotMatDefaultPIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=-1;
			RotMatDefaultDIP(1,2)=1;
			RotMatDefaultDIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=-1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.033, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.025, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC,wrist_joint,MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;
		}else{
			// Create DH parameter
			DHParameter DH_wrist_FE=DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.065, PI/2, -0.020 , std::pair <double,double>(-PI/9.0, PI/3.0));				// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.033, std::pair <double,double>(-PI/9.0, 3.0*PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.025, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.016, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=1;
			RotMatDefaultMCP(1,0)=1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=1;
			RotMatDefaultPIP(1,2)=-1;
			RotMatDefaultPIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=1;
			RotMatDefaultDIP(1,2)=-1;
			RotMatDefaultDIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.033, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.025, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC, wrist_joint, MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;
		}


	};

	SkeletonJoint Ring::getDip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[dip]];
		}else{
			return S.getGraph()[dip];
		}
	};

	SkeletonJoint Ring::getPip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[pip]];
		}else{
			return S.getGraph()[pip];
		}
	};

	SkeletonJoint Ring::getMcp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[mcp]];
		}else{
			return S.getGraph()[mcp];
		}
		
	};

	SkeletonJoint Ring::getTip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[tip]];
		}else{
			return S.getGraph()[tip];
		}
	};

	SkeletonBone Ring::getMC(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			SkeletonBone b=S.getGraph()[*ei];
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Ring::getPP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[pip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(pip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Ring::getMP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[dip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(dip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Ring::getDP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[tip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(tip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	Ring::~Ring(){
	};

	Middle::Middle(){
	};

	Middle::Middle(bool right){

		this->right=right;
		this->left=!right;
		double forearm_length=0.3;
		double boneLength_MC=sqrt(0.071*0.071 + 0.010*0.010);

		if(right){

			// Create DH parameter
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.071, PI/2, -0.010 , std::pair <double,double>(-PI/4.0, PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.035, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.026, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.018, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints


			// Create Wrist joint
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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=-1;
			RotMatDefaultMCP(1,0)=-1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=-1;
			RotMatDefaultPIP(1,2)=1;
			RotMatDefaultPIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=-1;
			RotMatDefaultDIP(1,2)=1;
			RotMatDefaultDIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=-1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.035, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.026, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.018, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC,wrist_joint,MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;

		}else{

			// Create DH parameter
			DHParameter DH_wrist_FE=DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.071, PI/2, -0.010 , std::pair <double,double>(-PI/9.0, PI/3.0));				// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.035, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.026, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.018, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=1;
			RotMatDefaultMCP(1,0)=1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=1;
			RotMatDefaultPIP(1,2)=-1;
			RotMatDefaultPIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=1;
			RotMatDefaultDIP(1,2)=-1;
			RotMatDefaultDIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.033, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.025, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC, wrist_joint, MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;

		}

	};

	SkeletonJoint Middle::getDip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[dip]];
		}else{
			return S.getGraph()[dip];
		}
	};

	SkeletonJoint Middle::getPip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[pip]];
		}else{
			return S.getGraph()[pip];
		}
	};

	SkeletonJoint Middle::getMcp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[mcp]];
		}else{
			return S.getGraph()[mcp];
		}
		
	};

	SkeletonJoint Middle::getTip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[tip]];
		}else{
			return S.getGraph()[tip];
		}
	};

	SkeletonBone Middle::getMC(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Middle::getPP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[pip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(pip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Middle::getMP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[dip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(dip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Middle::getDP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[tip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(tip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	Middle::~Middle(){
	};

	Index::Index(){
	};

	Index::Index(bool right){

		this->right=right;
		this->left=!right;
		double forearm_length=0.3;
		double boneLength_MC=sqrt(0.065*0.065 + 0.020*0.020);

		if(right){

			// Create DH parameter
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.065, PI/2, 0.020 , std::pair <double,double>(-PI/4.0, PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.030, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/2.0, PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.018, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

			// Create Wrist joint
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

			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=-1;
			RotMatDefaultMCP(1,0)=-1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=-1;
			RotMatDefaultPIP(1,2)=1;
			RotMatDefaultPIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=-1;
			RotMatDefaultDIP(1,2)=1;
			RotMatDefaultDIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=-1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.033, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.025, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC,wrist_joint,MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;

		}else{

			// Create DH parameter
			DHParameter DH_wrist_FE=DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.065, PI/2, 0.020 , std::pair <double,double>(-PI/9.0, PI/3.0));				// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/2.0  ,PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_PIP_FE=DHParameter(0, 0, 0, 0.030, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_DIP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/9.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.018, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints
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

			// create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA,AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE,FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=1;
			RotMatDefaultMCP(1,0)=1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	

			// Create PIP joint
			std::vector <JointDOF> PIP_DOF;
			PIP_DOF.push_back(JointDOF(DH_PIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultPIP(0,1)=1;
			RotMatDefaultPIP(1,2)=-1;
			RotMatDefaultPIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionPIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionPIP(0,0)=1;
			RotMatDefaultToUnionPIP(1,2)=-1;
			RotMatDefaultToUnionPIP(2,1)=1;
			SkeletonJoint PIP_joint=SkeletonJoint(PIP_DOF, PIP, RotMatDefaultPIP, RotMatDefaultToUnionPIP);	

			// Create DIP joint
			std::vector <JointDOF> DIP_DOF;
			DIP_DOF.push_back(JointDOF(DH_DIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultDIP(0,1)=1;
			RotMatDefaultDIP(1,2)=-1;
			RotMatDefaultDIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionDIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionDIP(0,0)=1;
			RotMatDefaultToUnionDIP(1,2)=-1;
			RotMatDefaultToUnionDIP(2,1)=1;
			SkeletonJoint DIP_joint=SkeletonJoint(DIP_DOF, DIP, RotMatDefaultDIP, RotMatDefaultToUnionDIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,1)=1;
			RotMatDefaultTIP(1,2)=-1;
			RotMatDefaultTIP(2,0)=1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,2)=-1;
			RotMatDefaultToUnionTIP(2,1)=1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF, TIP, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.033, 0.015, RotMatDefaultPIP, RotMatDefaultToUnionPIP);
			SkeletonBone MP=SkeletonBone(0.025, 0.015, RotMatDefaultDIP, RotMatDefaultToUnionDIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(PIP_joint);
			S_graph.addJoint(DIP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC, wrist_joint, MCP_joint);
			S_graph.addBone(PP,MCP_joint,PIP_joint);
			S_graph.addBone(MP,PIP_joint,DIP_joint);
			S_graph.addBone(DP,DIP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			pip=PIP_joint.getVertex();
			dip=DIP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;
		}

	};

	SkeletonJoint Index::getDip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[dip]];
		}else{
			return S.getGraph()[dip];
		}
	};

	SkeletonJoint Index::getPip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[pip]];
		}else{
			return S.getGraph()[pip];
		}
	};

	SkeletonJoint Index::getMcp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[mcp]];
		}else{
			return S.getGraph()[mcp];
		}
		
	};

	SkeletonJoint Index::getTip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[tip]];
		}else{
			return S.getGraph()[tip];
		}
	};

	SkeletonBone Index::getMC(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Index::getPP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[pip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(pip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Index::getMP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[dip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(dip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Index::getDP(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[tip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(tip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	Index::~Index(){
	};

	Thumb::Thumb(){
	};

	Thumb::Thumb(bool right){

		this->right=right;
		this->left=!right;
		double boneLength_MC=sqrt(0.025*0.025 + 0.017*0.017);
		double forearm_length=0.3;

		if(right){
			// Create DH parameter
			DHParameter DH_wrist_FE=Q2::DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=Q2::DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=Q2::DHParameter((5*PI/4), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.025, PI/2, 0.017 , std::pair <double,double>(-PI/4.0, PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/4.0 ,PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_CMC_AA=DHParameter(0, 0, 0, 0.025, std::pair <double,double>(-PI/18.0, 4.0*PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_CMC_FE=DHParameter(0, 0, PI/2.0, 0, std::pair <double,double>(-PI/4.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_IP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/4.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.016, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints

			// Create Wrist joint
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


			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA, AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE, FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=-1;
			RotMatDefaultMCP(1,0)=-1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	
	

			// Create CMC* joint
			std::vector <JointDOF> CMC_DOF;
			CMC_DOF.push_back(JointDOF(DH_CMC_AA, AA));
			CMC_DOF.push_back(JointDOF(DH_CMC_FE, FE));
			Eigen::Matrix3d RotMatDefaultCMC=Eigen::Matrix3d::Zero();
			RotMatDefaultCMC(0,1)=-1;
			RotMatDefaultCMC(1,2)=1;
			RotMatDefaultCMC(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnioCMC=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnioCMC(0,0)=1;
			RotMatDefaultToUnioCMC(1,2)=-1;
			RotMatDefaultToUnioCMC(2,1)= 1;
			SkeletonJoint CMC_joint=SkeletonJoint(CMC_DOF, CMC, RotMatDefaultCMC, RotMatDefaultToUnioCMC);	

			// Create IP joint
			std::vector <JointDOF> IP_DOF;
			IP_DOF.push_back(JointDOF(DH_IP_FE, FE));
			Eigen::Matrix3d RotMatDefaultIP=Eigen::Matrix3d::Zero();
			RotMatDefaultIP(0,2)=1;
			RotMatDefaultIP(1,1)=1;
			RotMatDefaultIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnioIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnioIP(0,0)=1;
			RotMatDefaultToUnioIP(1,1)=-1;
			RotMatDefaultToUnioIP(2,2)=-1;
			SkeletonJoint IP_joint=SkeletonJoint(IP_DOF, IP, RotMatDefaultIP, RotMatDefaultToUnioIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,2)=1;
			RotMatDefaultTIP(1,1)=1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,1)=-1;
			RotMatDefaultToUnionTIP(2,2)=-1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF,TIP,RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.025, 0.015, RotMatDefaultCMC, RotMatDefaultToUnioCMC);
			SkeletonBone MP=SkeletonBone(0.020, 0.015, RotMatDefaultIP, RotMatDefaultToUnioIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(CMC_joint);
			S_graph.addJoint(IP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC,wrist_joint,MCP_joint);
			S_graph.addBone(PP,MCP_joint,CMC_joint);
			S_graph.addBone(MP,CMC_joint,IP_joint);
			S_graph.addBone(DP,IP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			cmc=CMC_joint.getVertex();
			ip=IP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;

		}else{

			// Create DH parameter
			DHParameter DH_wrist_FE=DHParameter(PI/2, 0, 0, forearm_length, std::pair <double,double>(-(1.0/2.0)*PI,(1.0/2.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_AA=DHParameter(0, 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_wrist_PS=DHParameter((3*PI/2), 0, PI/2, 0, std::pair <double,double>(-(4.0/9.0)*PI,(4.0/9.0)*PI));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_AA=DHParameter(PI, -0.025, PI/2, 0.017 , std::pair <double,double>(-PI/9.0, PI/3.0));				// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_MCP_FE=DHParameter(3.0*PI/2.0, 0, PI/2, 0, std::pair <double,double>(-PI/4.0 ,PI/4.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_CMC_AA=DHParameter(0, 0, 0, 0.025, std::pair <double,double>(-PI/4.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_CMC_FE=DHParameter(0, 0, PI/2.0, 0, std::pair <double,double>(-PI/18.0, 4.0*PI/9.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_IP_FE=DHParameter(0, 0, 0, 0.020, std::pair <double,double>(-PI/4.0, PI/2.0));// phi, d, alpha, a, MinMaxPhi
			DHParameter DH_TIP_FE=DHParameter(0, 0, 0, 0.016, std::pair <double,double>(0.0, 0.0));// phi, d, alpha, a, MinMaxPhi

			// Create joints
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


			// Create MCP joint
			std::vector <JointDOF> MCP_DOF;
			MCP_DOF.push_back(JointDOF(DH_MCP_AA, AA));
			MCP_DOF.push_back(JointDOF(DH_MCP_FE, FE));
			Eigen::Matrix3d RotMatDefaultMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultMCP(0,2)=1;
			RotMatDefaultMCP(1,0)=1;
			RotMatDefaultMCP(2,1)=1;
			Eigen::Matrix3d RotMatDefaultToUnionMCP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionMCP(0,1)=-1;
			RotMatDefaultToUnionMCP(1,0)=1;
			RotMatDefaultToUnionMCP(2,2)=1;
			SkeletonJoint MCP_joint=SkeletonJoint(MCP_DOF,MCP,RotMatDefaultMCP, RotMatDefaultToUnionMCP);	
	

			// Create CMC* joint
			std::vector <JointDOF> CMC_DOF;
			CMC_DOF.push_back(JointDOF(DH_CMC_AA, AA));
			CMC_DOF.push_back(JointDOF(DH_CMC_FE, FE));
			Eigen::Matrix3d RotMatDefaultCMC=Eigen::Matrix3d::Zero();
			RotMatDefaultCMC(0,1)=-1;
			RotMatDefaultCMC(1,2)=1;
			RotMatDefaultCMC(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnioCMC=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnioCMC(0,0)=1;
			RotMatDefaultToUnioCMC(1,2)=-1;
			RotMatDefaultToUnioCMC(2,1)= 1;
			SkeletonJoint CMC_joint=SkeletonJoint(CMC_DOF, CMC, RotMatDefaultCMC, RotMatDefaultToUnioCMC);	

			// Create IP joint
			std::vector <JointDOF> IP_DOF;
			IP_DOF.push_back(JointDOF(DH_IP_FE, FE));
			Eigen::Matrix3d RotMatDefaultIP=Eigen::Matrix3d::Zero();
			RotMatDefaultIP(0,2)=1;
			RotMatDefaultIP(1,1)=1;
			RotMatDefaultIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnioIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnioIP(0,0)=1;
			RotMatDefaultToUnioIP(1,1)=-1;
			RotMatDefaultToUnioIP(2,2)=-1;
			SkeletonJoint IP_joint=SkeletonJoint(IP_DOF, IP, RotMatDefaultIP, RotMatDefaultToUnioIP);

			// Create TIP joint
			std::vector <JointDOF> TIP_DOF;
			TIP_DOF.push_back(JointDOF(DH_TIP_FE,FE));
			Eigen::Matrix3d RotMatDefaultTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultTIP(0,2)=1;
			RotMatDefaultTIP(1,1)=1;
			RotMatDefaultTIP(2,0)=-1;
			Eigen::Matrix3d RotMatDefaultToUnionTIP=Eigen::Matrix3d::Zero();
			RotMatDefaultToUnionTIP(0,0)=1;
			RotMatDefaultToUnionTIP(1,1)=-1;
			RotMatDefaultToUnionTIP(2,2)=-1;
			SkeletonJoint TIP_joint=SkeletonJoint(TIP_DOF,TIP,RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create bones
			SkeletonBone MC=SkeletonBone(boneLength_MC, 0.015, RotMatDefaultMCP, RotMatDefaultToUnionMCP);
			SkeletonBone PP=SkeletonBone(0.025, 0.015, RotMatDefaultCMC, RotMatDefaultToUnioCMC);
			SkeletonBone MP=SkeletonBone(0.020, 0.015, RotMatDefaultIP, RotMatDefaultToUnioIP);
			SkeletonBone DP=SkeletonBone(0.016, 0.015, RotMatDefaultTIP, RotMatDefaultToUnionTIP);

			// Create Graph
			Skeleton S_graph;

			// Add joints to graph
			S_graph.addJoint(wrist_joint);
			S_graph.addJoint(MCP_joint);
			S_graph.addJoint(CMC_joint);
			S_graph.addJoint(IP_joint);
			S_graph.addJoint(TIP_joint);

			// Add bones to graph
			S_graph.addBone(MC, wrist_joint, MCP_joint);
			S_graph.addBone(PP,MCP_joint,CMC_joint);
			S_graph.addBone(MP,CMC_joint,IP_joint);
			S_graph.addBone(DP,IP_joint,TIP_joint);

			// Set global position and global orientation
			Eigen::Vector3d PositionGlobal(0, 0, 0);
			Eigen::Matrix3d RotMatGlobal = Eigen::Matrix3d::Identity();
			S_graph.setStartJoint(MCP_joint, PositionGlobal, RotMatGlobal);

			// Set vertices
			wrist=wrist_joint.getVertex();
			mcp=MCP_joint.getVertex();
			cmc=CMC_joint.getVertex();
			ip=IP_joint.getVertex();
			tip=TIP_joint.getVertex();

			// Set edges
			mc=MC.getEdge();
			pp=PP.getEdge();
			mp=MP.getEdge();
			dp=DP.getEdge();

			// Set Graph
			S=S_graph;
		}
	};

	SkeletonJoint Thumb::getMcp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[mcp]];
		}else{
			return S.getGraph()[mcp];
		}
	};

	SkeletonJoint Thumb::getCmc(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[cmc]];
		}else{
			return S.getGraph()[cmc];
		}
	};

	SkeletonJoint Thumb::getIp(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[ip]];
		}else{
			return S.getGraph()[ip];
		}
	};

	SkeletonJoint Thumb::getTip(){
		if(S.getMapping()){
			return S.getMappingSkeleton()->getGraph()[S.getMap()[tip]];
		}else{
			return S.getGraph()[tip];
		}
	};

	SkeletonBone Thumb::getMC(){
		in_edge_iter ei;
		in_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::in_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::in_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Thumb::getPP(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::out_edges(S.getMap()[mcp], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::out_edges(mcp, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Thumb::getMP(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::out_edges(S.getMap()[cmc], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::out_edges(cmc, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	SkeletonBone Thumb::getDP(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		if(S.getMapping()){
			boost::tie(ei, ei_end) = boost::out_edges(S.getMap()[ip], S.getMappingSkeleton()->getGraph());
			return S.getGraph()[*ei];
		}else{
			boost::tie(ei, ei_end) = boost::out_edges(ip, S.getGraph());
			return S.getGraph()[*ei];
		}
	};

	Thumb::~Thumb(){
	};
	
}


