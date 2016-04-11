/**
@file HandArm.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/


#include "HandArm.h"
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
//#include "HandArmA5.h"


const double PI = boost::math::constants::pi<double>();

namespace Q2{

	//using namespace Q2;

	HandArm::HandArm(){
		id=0;
	};

	HandArm::HandArm(Q2::Arm arm, Q2::Pinky pinky, Q2::Ring ring, Q2::Middle middle, Q2::Index index, Q2::Thumb thumb, bool isRight, int32_t id){

		clock_t begin = clock();

		this->isRight=isRight;
		this->isLeft=!isRight;

		if(isRight){
			// Add arm and pinky to HandArm model
			Q2::JointVertex v1=2;
			Q2::JointVertex v2=0;
			this->mergeSkeletonbyJointVertex(arm.getSkeleton(), v1, pinky.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, ring.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, middle.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, index.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, thumb.getSkeleton(), v2);

			this->arm=arm;
			this->pinky=pinky;
			this->ring=ring;
			this->middle=middle;
			this->index=index;
			this->thumb=thumb;
			this->id=id;

			// set Startjoint
			JointVertex v=2;
			this->S.setStartJoint(v);

		}else{

			
			Q2::JointVertex v1=2;
			Q2::JointVertex v2=0;
			this->mergeSkeletonbyJointVertex(arm.getSkeleton(), v1, pinky.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, ring.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, middle.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, index.getSkeleton(), v2);
			this->mergeSkeletonbyJointVertex(this->getSkeleton(), v1, thumb.getSkeleton(), v2);

			double elapsed_secs3 = double(clock() - begin) / CLOCKS_PER_SEC;

			this->arm=arm;
			this->pinky=pinky;
			this->ring=ring;
			this->middle=middle;
			this->index=index;
			this->thumb=thumb;
			this->id=id;

			// set Startjoint
			JointVertex v=2;
			this->S.setStartJoint(v);
		}
	};

	// set body parts
	void HandArm::setArm(const Arm arm){
		this->arm=arm;
	};

	void HandArm::setPinky(const Pinky pinky){
		this->pinky=pinky;
	};
	
	void HandArm::setRing(const Ring ring){
		this->ring=ring;
	};
	
	void HandArm::setMiddle(const Middle middle){
		this->middle=middle;
	};
	
	void HandArm::setIndex(const Index index){
		this->index=index;
	};

	void HandArm::setThumb(const Thumb thumb){
		this->thumb=thumb;
	};

	Arm& HandArm::getArm(){
		return arm;
	};

	Pinky& HandArm::getPinky(){
		return pinky;
	};

	Ring& HandArm::getRing(){
		return ring;
	};

	Middle& HandArm::getMiddle(){
		return middle;
	};

	Index& HandArm::getIndex(){
		return index;
	};
	
	Thumb& HandArm::getThumb(){
		return thumb;
	};

	void HandArm::setSkeleton(Skeleton S){
		this->S=S;
	};

	void HandArm::setTimestamp(int64_t frametimestamp){
		this->frametimestamp=frametimestamp;
	};

	void HandArm::setIsLeft(bool isLeft){
		this->isLeft=isLeft;
		this->isRight=!isLeft;
	};

	void HandArm::setID(int32_t id){
		this->id=id;
	}

	Skeleton HandArm::getSkeleton(){
		return this->S;
	};

	void HandArm::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(joint, PositionGlobal, RotMatGlobal);
	};

	void HandArm::setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		S.setStartJoint(vertex, PositionGlobal, RotMatGlobal);
	};

	bool HandArm::update(SkeletonJoint &joint, int DOF, double phi_t, double score){
		return S.update(joint, DOF, phi_t, score);
	};

	bool HandArm::update(JointVertex &vertex, int DOF, double phi_t, double score){
		return S.update(vertex, DOF, phi_t, score);
	};
	
	bool HandArm::update(){
		return S.update();
	};

	bool HandArm::updateParts(){

		return true;
	};

	bool HandArm::printModel(){

		// print results
		std::ofstream myfile;
		// open file
		myfile.open ("..\\data\\log.txt",std::ofstream::app);
		myfile << "" << std::endl;

		// print timestamp
		myfile << "New Frame: " << this->timestamp() <<std::endl;

		// print joint angles from arm
		myfile << "joint angles: " << std::endl;
		myfile << " phi_shoulder_AA: " << this->getArm().getShoulder().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_shoulder_FE: " << this->getArm().getShoulder().getDOFvec()[1].getDHParameter().getPhi_t() << ", phi_shoulder_PS: " << this->getArm().getShoulder().getDOFvec()[2].getDHParameter().getPhi_t()<< std::endl;
		myfile << " phi_elbow_FE: " << this->getArm().getElbow().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_wrist_AA: " << this->getArm().getWrist().getDOFvec()[0].getDHParameter().getPhi_t() << " phi_wrist_FE: " << this->getArm().getWrist().getDOFvec()[1].getDHParameter().getPhi_t() << ", phi_wrist_PS: " << this->getArm().getWrist().getDOFvec()[2].getDHParameter().getPhi_t()<< std::endl;
		
		// print joint angles from pinky
		myfile << " phi_pinky_MCP_AA: " << this->getPinky().getMcp().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_pinky_MCP_FE: " << this->getPinky().getMcp().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_pinky_PIP_FE: " << this->getPinky().getPip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_pinky_DIP_FE: " << this->getPinky().getDip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		
		// print joint angles from ring
		myfile << " phi_ring_MCP_AA: " << this->getRing().getMcp().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_ringP_FE: " << this->getRing().getMcp().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_ring_PIP_FE: " << this->getRing().getPip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_ring_DIP_FE: " << this->getRing().getDip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;		

		// print joint angles from middle
		myfile << " phi_middle_MCP_AA: " << this->getMiddle().getMcp().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_middle_MCP_FE: " << this->getMiddle().getMcp().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_middle_PIP_FE: " << this->getMiddle().getPip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_middle_DIP_FE: " << this->getMiddle().getDip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;	

		// print joint angles from index
		myfile << " phi_index_MCP_AA: " << this->getIndex().getMcp().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_index_MCP_FE: " << this->getIndex().getMcp().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_index_PIP_FE: " << this->getIndex().getPip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_index_DIP_FE: " << this->getIndex().getDip().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;	

		// print joint angles from thumb
		myfile << " phi_thumb_MCP_AA: " << this->getThumb().getMcp().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_index_MCP_FE: " << this->getThumb().getMcp().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_thumb_Cmc_AA: " << this->getThumb().getCmc().getDOFvec()[0].getDHParameter().getPhi_t() << ", phi_thumb_Cmc_FE: " << this->getThumb().getCmc().getDOFvec()[1].getDHParameter().getPhi_t() << std::endl;
		myfile << " phi_thumb_IP_FE: " << this->getThumb().getIp().getDOFvec()[0].getDHParameter().getPhi_t() << std::endl;	

		// print bone postions in global coordinate system

		myfile << " Position_shoulder: " << this->getArm().getShoulder().getPositionGlobal()[0] << " " << this->getArm().getShoulder().getPositionGlobal()[1] << " " << this->getArm().getShoulder().getPositionGlobal()[2] << std::endl;	
		myfile << " Position_elbow: " << this->getArm().getElbow().getPositionGlobal()[0] << " " << this->getArm().getElbow().getPositionGlobal()[1] << " " << this->getArm().getElbow().getPositionGlobal()[2] << std::endl;	
		myfile << " Position_wrist: " << this->getArm().getWrist().getPositionGlobal()[0] << " " << this->getArm().getWrist().getPositionGlobal()[1] << " " << this->getArm().getWrist().getPositionGlobal()[2] << std::endl;	

		myfile << " Position_pinky_MC: " << this->getPinky().getMC().getPositionGlobal()[0] << " " << this->getPinky().getMC().getPositionGlobal()[1] << " " << this->getPinky().getMC().getPositionGlobal()[2] << std::endl;	
		myfile << " Position_pinky_PP: " << this->getPinky().getPP().getPositionGlobal()[0] << " " << this->getPinky().getPP().getPositionGlobal()[1] << " " << this->getPinky().getPP().getPositionGlobal()[2] << std::endl;	
		myfile << " Position_pinky_MP: " << this->getPinky().getMP().getPositionGlobal()[0] << " " << this->getPinky().getMP().getPositionGlobal()[1] << " " << this->getPinky().getMP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_pinky_DP: " << this->getPinky().getDP().getPositionGlobal()[0] << " " << this->getPinky().getDP().getPositionGlobal()[1] << " " << this->getPinky().getDP().getPositionGlobal()[2] << std::endl;

		myfile << " Position_ring_MC: " << this->getRing().getMC().getPositionGlobal()[0] << " " << this->getRing().getMC().getPositionGlobal()[1] << " " << this->getRing().getMC().getPositionGlobal()[2] << std::endl;
		myfile << " Position_ring_PP: " << this->getRing().getPP().getPositionGlobal()[0] << " " << this->getRing().getPP().getPositionGlobal()[1] << " " << this->getRing().getPP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_ring_MP: " << this->getRing().getMP().getPositionGlobal()[0] << " " << this->getRing().getMP().getPositionGlobal()[1] << " " << this->getRing().getMP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_ring_DP: " << this->getRing().getDP().getPositionGlobal()[0] << " " << this->getRing().getDP().getPositionGlobal()[1] << " " << this->getRing().getDP().getPositionGlobal()[2] << std::endl;

		myfile << " Position_Middle_MC: " << this->getMiddle().getMC().getPositionGlobal()[0] << " " << this->getMiddle().getMC().getPositionGlobal()[1] << " " << this->getMiddle().getMC().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Middle_PP: " << this->getMiddle().getPP().getPositionGlobal()[0] << " " << this->getMiddle().getPP().getPositionGlobal()[1] << " " << this->getMiddle().getPP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Middle_MP: " << this->getMiddle().getMP().getPositionGlobal()[0] << " " << this->getMiddle().getMP().getPositionGlobal()[1] << " " << this->getMiddle().getMP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Middle_DP: " << this->getMiddle().getDP().getPositionGlobal()[0] << " " << this->getMiddle().getDP().getPositionGlobal()[1] << " " << this->getMiddle().getDP().getPositionGlobal()[2] << std::endl;

		myfile << " Position_Index_MC: " << this->getIndex().getMC().getPositionGlobal()[0] << " " << this->getIndex().getMC().getPositionGlobal()[1] << " " << this->getIndex().getMC().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Index_PP: " << this->getIndex().getPP().getPositionGlobal()[0] << " " << this->getIndex().getPP().getPositionGlobal()[1] << " " << this->getIndex().getPP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Index_MP: " << this->getIndex().getMP().getPositionGlobal()[0] << " " << this->getIndex().getMP().getPositionGlobal()[1] << " " << this->getIndex().getMP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Index_DP: " << this->getIndex().getDP().getPositionGlobal()[0] << " " << this->getIndex().getDP().getPositionGlobal()[1] << " " << this->getIndex().getDP().getPositionGlobal()[2] << std::endl;

		myfile << " Position_Thumb_MC: " << this->getThumb().getMC().getPositionGlobal()[0] << " " << this->getThumb().getMC().getPositionGlobal()[1] << " " << this->getThumb().getMC().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Thumb_PP: " << this->getThumb().getPP().getPositionGlobal()[0] << " " << this->getThumb().getPP().getPositionGlobal()[1] << " " << this->getThumb().getPP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Thumb_MP: " << this->getThumb().getMP().getPositionGlobal()[0] << " " << this->getThumb().getMP().getPositionGlobal()[1] << " " << this->getThumb().getMP().getPositionGlobal()[2] << std::endl;
		myfile << " Position_Thumb_DP: " << this->getThumb().getDP().getPositionGlobal()[0] << " " << this->getThumb().getDP().getPositionGlobal()[1] << " " << this->getThumb().getDP().getPositionGlobal()[2] << std::endl;

		// print bone quaternion in union coordinate system
		myfile << " QuaternionUnion_Pinky_MC: " << "x: " << this->getPinky().getMC().getQuaternionUnion().x() << "y: " << this->getPinky().getMC().getQuaternionUnion().y() << " z: " << this->getPinky().getMC().getQuaternionUnion().z() << " w: " << this->getPinky().getMC().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Pinky_PP: " << "x: " << this->getPinky().getPP().getQuaternionUnion().x() << " y: " << this->getPinky().getPP().getQuaternionUnion().y() << " z: " << this->getPinky().getPP().getQuaternionUnion().z() << " w: " << this->getPinky().getPP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Pinky_MP: " << "x: " << this->getPinky().getMP().getQuaternionUnion().x() << " y: " << this->getPinky().getMP().getQuaternionUnion().y() << " z: " << this->getPinky().getMP().getQuaternionUnion().z() << " w: " << this->getPinky().getMP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Pinky_DP: " << "x: " << this->getPinky().getDP().getQuaternionUnion().x() << " y: " << this->getPinky().getDP().getQuaternionUnion().y() << " z: " << this->getPinky().getDP().getQuaternionUnion().z() << " w: " << this->getPinky().getDP().getQuaternionUnion().w() << std::endl;	

		myfile << " QuaternionUnion_Ring_MC: " << "x: " << this->getRing().getMC().getQuaternionUnion().x() << " y: " << this->getRing().getMC().getQuaternionUnion().y() << " z: " << this->getRing().getMC().getQuaternionUnion().z() << " w: " << this->getRing().getMC().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Ring_PP: " << "x: " << this->getRing().getPP().getQuaternionUnion().x() << " y: " << this->getRing().getPP().getQuaternionUnion().y() << " z: " << this->getRing().getPP().getQuaternionUnion().z() << " w: " << this->getRing().getPP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Ring_MP: " << "x: " << this->getRing().getMP().getQuaternionUnion().x() << " y: " << this->getRing().getMP().getQuaternionUnion().y() << " z: " << this->getRing().getMP().getQuaternionUnion().z() << " w: " << this->getRing().getMP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Ring_DP: " << "x: " << this->getRing().getDP().getQuaternionUnion().x() << " y: " << this->getRing().getDP().getQuaternionUnion().y() << " z: " << this->getRing().getDP().getQuaternionUnion().z() << " w: " << this->getRing().getDP().getQuaternionUnion().w() << std::endl;	

		myfile << " QuaternionUnion_Middle_MC: " << "x: " << this->getMiddle().getMC().getQuaternionUnion().x() << " y: " << this->getMiddle().getMC().getQuaternionUnion().y() << " z: " << this->getMiddle().getMC().getQuaternionUnion().z() << " w: " << this->getMiddle().getMC().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Middle_PP: " << "x: " << this->getMiddle().getPP().getQuaternionUnion().x() << " y: " << this->getMiddle().getPP().getQuaternionUnion().y() << " z: " << this->getMiddle().getPP().getQuaternionUnion().z() << " w: " << this->getMiddle().getPP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Middle_MP: " << "x: " << this->getMiddle().getMP().getQuaternionUnion().x() << " y: " << this->getMiddle().getMP().getQuaternionUnion().y() << " z: " << this->getMiddle().getMP().getQuaternionUnion().z() << " w: " << this->getMiddle().getMP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Middle_DP: " << "x: " << this->getMiddle().getDP().getQuaternionUnion().x() << " y: " << this->getMiddle().getDP().getQuaternionUnion().y() << " z: " << this->getMiddle().getDP().getQuaternionUnion().z() << " w: " << this->getMiddle().getDP().getQuaternionUnion().w() << std::endl;	

		myfile << " QuaternionUnion_Index_MC: " << "x: " << this->getIndex().getMC().getQuaternionUnion().x() << " y: " << this->getIndex().getMC().getQuaternionUnion().y() << " z: " << this->getIndex().getMC().getQuaternionUnion().z() << " w: " << this->getIndex().getMC().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Index_PP: " << "x: " << this->getIndex().getPP().getQuaternionUnion().x() << " y: " << this->getIndex().getPP().getQuaternionUnion().y() << " z: " << this->getIndex().getPP().getQuaternionUnion().z() << " w: " << this->getIndex().getPP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Index_MP: " << "x: " << this->getIndex().getMP().getQuaternionUnion().x() << " y: " << this->getIndex().getMP().getQuaternionUnion().y() << " z: " << this->getIndex().getMP().getQuaternionUnion().z() << " w: " << this->getIndex().getMP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Index_DP: " << "x: " << this->getIndex().getDP().getQuaternionUnion().x() << " y: " << this->getIndex().getDP().getQuaternionUnion().y() << " z: " << this->getIndex().getDP().getQuaternionUnion().z() << " w: " << this->getIndex().getDP().getQuaternionUnion().w() << std::endl;	

		myfile << " QuaternionUnion_Thumb_MC: " << "x: " << this->getThumb().getMC().getQuaternionUnion().x() << " y: " << this->getThumb().getMC().getQuaternionUnion().y() << " z: " << this->getThumb().getMC().getQuaternionUnion().z() << " w: " << this->getThumb().getMC().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Thumb_PP: " << "x: " << this->getThumb().getPP().getQuaternionUnion().x() << " y: " << this->getThumb().getPP().getQuaternionUnion().y() << " z: " << this->getThumb().getPP().getQuaternionUnion().z() << " w: " << this->getThumb().getPP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Thumb_MP: " << "x: " << this->getThumb().getMP().getQuaternionUnion().x() << " y: " << this->getThumb().getMP().getQuaternionUnion().y() << " z: " << this->getThumb().getMP().getQuaternionUnion().z() << " w: " << this->getThumb().getMP().getQuaternionUnion().w() << std::endl;	
		myfile << " QuaternionUnion_Thumb_DP: " << "x: " << this->getThumb().getDP().getQuaternionUnion().x() << " y: " << this->getThumb().getDP().getQuaternionUnion().y() << " z: " << this->getThumb().getDP().getQuaternionUnion().z() << " w: " << this->getThumb().getDP().getQuaternionUnion().w() << std::endl;	
		
		// print joint scores from arm
		myfile << "scores: " << std::endl;
		myfile << " score_shoulder_AA: " << this->getArm().getShoulder().getDOFvec()[0].getScore() << ", phi_shoulder_FE: " << this->getArm().getShoulder().getDOFvec()[1].getScore() << ", phi_shoulder_PS: " << this->getArm().getShoulder().getDOFvec()[2].getScore() << std::endl;
		myfile << " score_elbow_FE: " << this->getArm().getElbow().getDOFvec()[0].getScore() << std::endl;
		myfile << " score_wrist_AA: " << this->getArm().getWrist().getDOFvec()[0].getScore() << " phi_wrist_FE: " << this->getArm().getWrist().getDOFvec()[1].getScore() << ", phi_wrist_PS: " << this->getArm().getWrist().getDOFvec()[2].getScore()<< std::endl;

		// print joint scores from pinky
		myfile << " score_pinky_MCP_AA: " << this->getPinky().getMcp().getDOFvec()[0].getScore() << ", phi_pinky_MCP_FE: " << this->getPinky().getMcp().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_pinky_PIP_FE: " << this->getPinky().getPip().getDOFvec()[0].getScore() << std::endl;
		myfile << " score_pinky_DIP_FE: " << this->getPinky().getDip().getDOFvec()[0].getScore() << std::endl;

		// print joint angles from ring
		myfile << " score_ring_MCP_AA: " << this->getRing().getMcp().getDOFvec()[0].getScore() << ", phi_ringP_FE: " << this->getRing().getMcp().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_ring_PIP_FE: " << this->getRing().getPip().getDOFvec()[0].getScore() << std::endl;
		myfile << " score_ring_DIP_FE: " << this->getRing().getDip().getDOFvec()[0].getScore() << std::endl;		

		// print joint angles from middle
		myfile << " score_middle_MCP_AA: " << this->getMiddle().getMcp().getDOFvec()[0].getScore() << ", phi_middle_MCP_FE: " << this->getMiddle().getMcp().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_middle_PIP_FE: " << this->getMiddle().getPip().getDOFvec()[0].getScore() << std::endl;
		myfile << " score_middle_DIP_FE: " << this->getMiddle().getDip().getDOFvec()[0].getScore() << std::endl;	

		// print joint angles from index
		myfile << " score_index_MCP_AA: " << this->getIndex().getMcp().getDOFvec()[0].getScore() << ", phi_index_MCP_FE: " << this->getIndex().getMcp().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_index_PIP_FE: " << this->getIndex().getPip().getDOFvec()[0].getScore() << std::endl;
		myfile << " score_index_DIP_FE: " << this->getIndex().getDip().getDOFvec()[0].getScore() << std::endl;	

		// print joint angles from thumb
		myfile << " score_thumb_MCP_AA: " << this->getThumb().getMcp().getDOFvec()[0].getScore() << ", phi_index_MCP_FE: " << this->getThumb().getMcp().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_thumb_Cmc_AA: " << this->getThumb().getCmc().getDOFvec()[0].getScore() << ", phi_thumb_Cmc_FE: " << this->getThumb().getCmc().getDOFvec()[1].getScore() << std::endl;
		myfile << " score_thumb_IP_FE: " << this->getThumb().getIp().getDOFvec()[0].getScore() << std::endl;	

		myfile.close();

	
		return true;
	};

	void HandArm::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone){		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		this->S.mergeSkeletonbyJointVertex(S1, v1, S2, v2, DH, bone);
	};

	void HandArm::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2){		// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone
		this->S.mergeSkeletonbyJointVertex(S1, v1, S2, v2);
	};

	SkeletonJoint HandArm::getShoulder(){
		return S.getGraph()[shoulder];
	};

	SkeletonJoint HandArm::getElbow(){
		return S.getGraph()[elbow];
	};

	SkeletonJoint HandArm::getWrist(){
		return S.getGraph()[wrist];
	};


	SkeletonBone HandArm::getUpperarm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(shoulder, g);
		return S.getGraph()[*ei];
	};

	SkeletonBone HandArm::getForearm(){
		out_edge_iter ei;
		out_edge_iter ei_end;
		SkeletonGraph g=S.getGraph();
		boost::tie(ei, ei_end) = boost::out_edges(elbow, g);
		return S.getGraph()[*ei];
	};

	bool HandArm::getIsLeft(){
		return isLeft;
	};

	bool HandArm::getIsRight(){
		return isRight;
	};

	int32_t HandArm::getID(){
		return id;
	}


	int64_t HandArm::timestamp(){
		return frametimestamp;
	};

	HandArm::~HandArm(){
	};

}


