/**
@file LeapToQ2.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/
#include "LeapToQ2.h"




/** @brief Namespace LeapToQ2
*/
namespace LeapToQ2
{

	#ifndef PI
	#define PI    3.14159265358979323846f
	#endif

	Eigen::Matrix3d LeapToQ2Rot(){
		Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
		T(0,2)=-1;
		T(1,0)=-1;
		T(2,1)=1;
		return T;
	}

	const Eigen::Matrix3d T4_forearm_right(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,2)=-1;
		T4(1,1)=-1;
		T4(2,0)=-1;
		return T4;
	}

	const Eigen::Matrix3d T4_forearm_left(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,2)=-1;
		T4(1,1)=1;
		T4(2,0)=-1;
		return T4;
	}

	const Eigen::Matrix3d T4_hand(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,0)=-1;
		T4(1,2)=1;
		T4(2,1)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Pinky_MC_right(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,0)=-1;
		T4(1,2)=1;
		T4(2,1)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Pinky_MC_left(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,0)=-1;
		T4(1,2)=1;
		T4(2,1)=-1;
		return T4;
	}

	const Eigen::Matrix3d T4_Pinky_PP(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,2)=-1;
		T4(1,1)=1;
		T4(2,0)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Pinky_MP(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,2)=-1;
		T4(1,1)=1;
		T4(2,0)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Pinky_DP(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,2)=-1;
		T4(1,1)=1;
		T4(2,0)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Finger_right(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,0)=-1;
		T4(1,2)=1;
		T4(2,1)=1;
		return T4;
	}

	const Eigen::Matrix3d T4_Finger_left(){
		Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		T4(0,0)=-1;
		T4(1,2)=1;
		T4(2,1)=-1;
		return T4;
	}

	void LeapRotationBone(Leap::Bone &bone, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation, bool right){

		Leap::Vector boneCenter = bone.center();

		// Transformation from Leap-bone coordinate system to leap-global coordinate system
		Eigen::Matrix3d T3;	
		Leap::Matrix LeapBasis=bone.basis();
		Leap::Vector xBasis = LeapBasis.xBasis;
		Leap::Vector yBasis = LeapBasis.yBasis;
		Leap::Vector zBasis = LeapBasis.zBasis;

		Leap::Matrix T3L = Leap::Matrix(xBasis, yBasis, zBasis, boneCenter);
		xBasis = T3L.xBasis;
		yBasis = T3L.yBasis;
		zBasis = T3L.zBasis;
		T3(0,0) = xBasis[0];
		T3(1,0) = xBasis[1];
		T3(2,0) = xBasis[2];
		T3(0,1) = yBasis[0];
		T3(1,1) = yBasis[1];
		T3(2,1) = yBasis[2];
		T3(0,2) = zBasis[0];
		T3(1,2) = zBasis[1];
		T3(2,2) = zBasis[2];

		// Transformation from Leap-bone coordinate system to Default-bone coordinate system
		Eigen::Matrix3d T4;
		
		if(right){
			T4=T4_Finger_right();
		}else{
			T4=T4_Finger_left();
		}
		/*
		switch (bone.type())
		{
			case Leap::Bone::TYPE_METACARPAL:
				if(right){
					T4=T4_Finger_right();
				}else{
					T4=T4_Pinky_MC_left();
				}
				break;
			case Leap::Bone::TYPE_PROXIMAL:
				T4=T4_Pinky_PP();
				break;
			case Leap::Bone::TYPE_INTERMEDIATE:
				T4=T4_Pinky_MP();
				break;
			case Leap::Bone::TYPE_DISTAL:
				T4=T4_Pinky_DP();
				break;
		}
		*/

		// Transformation from Leap coordinate system to Q2 coordinate system
		Eigen::Matrix3d T1=LeapToQ2::LeapToQ2Rot();

		// Bone default transformation 
		Eigen::Matrix3d T2=joint.getRotMatDefault();

		// calculate bone rotation matrix
		Eigen::Matrix3d R;
		R=T4*T3.inverse()*T1.inverse()*T2;
		Rotation=T2*R.inverse();
	};

	void LeapRotationArm(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation){
		
		Leap::Vector armCenter = arm.elbowPosition() + (arm.wristPosition() - arm.elbowPosition()) * .5;
		
		// Transformation from Leap-bone coordinate system to leap-global coordinate system
		Eigen::Matrix3d T3;	
		Leap::Matrix LeapBasis=arm.basis();
		Leap::Vector xBasis = LeapBasis.xBasis;
		Leap::Vector yBasis = LeapBasis.yBasis;
		Leap::Vector zBasis = LeapBasis.zBasis;

		Leap::Matrix T3L = Leap::Matrix(xBasis, yBasis, zBasis, armCenter);
		xBasis = T3L.xBasis;
		yBasis = T3L.yBasis;
		zBasis = T3L.zBasis;
		T3(0,0) = xBasis[0];
		T3(1,0) = xBasis[1];
		T3(2,0) = xBasis[2];
		T3(0,1) = yBasis[0];
		T3(1,1) = yBasis[1];
		T3(2,1) = yBasis[2];
		T3(0,2) = zBasis[0];
		T3(1,2) = zBasis[1];
		T3(2,2) = zBasis[2];

		// Transformation from Leap-bone coordinate system to Default-bone coordinate system
		Eigen::Matrix3d T4=Eigen::Matrix3d::Identity();
		if(hand.isRight()){
			T4=T4_forearm_right();
		}else{
			T4=T4_forearm_left();
		}

		// Transformation from Leap coordinate system to Q2 coordinate system
		Eigen::Matrix3d T1=LeapToQ2::LeapToQ2Rot();

		// Bone default transformation 
		Eigen::Matrix3d T2=joint.getRotMatDefault();

		// calculate bone rotation matrix
		Eigen::Matrix3d R;
		R=T4*T3.inverse()*T1.inverse()*T2;
		Rotation=T2*R.inverse();

	};

	/*
	void LeapRotationHand(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation){
		
		Leap::Vector armCenter = arm.elbowPosition() + (arm.wristPosition() - arm.elbowPosition()) * .5;
		//Leap::Matrix ArmBasis=arm.basis();
		// Transformation from Leap-bone coordinate system to leap-global coordinate system

		Eigen::Matrix3d T3;	
		Leap::Matrix LeapBasis=hand.basis();
		Leap::Vector xBasis = LeapBasis.xBasis;
		Leap::Vector yBasis = LeapBasis.yBasis;
		Leap::Vector zBasis = LeapBasis.zBasis;

		Leap::Matrix T3L = Leap::Matrix(xBasis, yBasis, zBasis, armCenter);
		xBasis = T3L.xBasis;
		yBasis = T3L.yBasis;
		zBasis = T3L.zBasis;
		T3(0,0) = xBasis[0];
		T3(1,0) = xBasis[1];
		T3(2,0) = xBasis[2];
		T3(0,1) = yBasis[0];
		T3(1,1) = yBasis[1];
		T3(2,1) = yBasis[2];
		T3(0,2) = zBasis[0];
		T3(1,2) = zBasis[1];
		T3(2,2) = zBasis[2];

		// Transformation from Leap-bone coordinate system to Default-bone coordinate system
		//Eigen::Matrix3d T4=Eigen::Matrix3d::Zero(); 
		//T4(0,2)=-1;
		//T4(1,1)=-1;
		//T4(2,0)=-1;
		const Eigen::Matrix3d T4=T4_forearm_right();


		// Transformation from Leap coordinate system to Q2 coordinate system
		Eigen::Matrix3d T1=LeapToQ2::LeapToQ2Rot();

		// Bone default transformation 
		Eigen::Matrix3d T2=joint.getRotMatDefault();

		// calculate bone rotation matrix
		Eigen::Matrix3d R;
		R=T4*T3.inverse()*T1.inverse()*T2;
		Rotation=T2*R.inverse();
	};
	*/


	/*
	void LeapRotationHand(Leap::Arm &arm, Leap::Hand &hand, Q2::SkeletonJoint &joint, Eigen::Matrix3d &Rotation){
				
		
		Leap::FingerList middelFingerList = hand.fingers().fingerType(Leap::Finger::TYPE_MIDDLE);
		Leap::Vector tipposition;
		tipposition=(*middelFingerList.begin()).tipPosition();
		Leap::Vector middelhandCenter = arm.wristPosition() + (tipposition - arm.wristPosition()) * .5;

		// Transformation from Leap-bone coordinate system to leap-global coordinate system
		Eigen::Matrix3d T3;	
		Leap::Matrix LeapBasis=hand.basis();
		Leap::Vector xBasis = LeapBasis.xBasis;
		Leap::Vector yBasis = LeapBasis.yBasis;
		Leap::Vector zBasis = LeapBasis.zBasis;

		Leap::Matrix T3L = Leap::Matrix(xBasis, yBasis, zBasis, middelhandCenter);
		xBasis = T3L.xBasis;
		yBasis = T3L.yBasis;
		zBasis = T3L.zBasis;
		T3(0,0) = xBasis[0];
		T3(1,0) = xBasis[1];
		T3(2,0) = xBasis[2];
		T3(0,1) = yBasis[0];
		T3(1,1) = yBasis[1];
		T3(2,1) = yBasis[2];
		T3(0,2) = zBasis[0];
		T3(1,2) = zBasis[1];
		T3(2,2) = zBasis[2];

		// Transformation from Leap-bone coordinate system to Default-bone coordinate system
		const Eigen::Matrix3d T4=T4_hand();

		// Transformation from Leap coordinate system to Q2 coordinate system
		Eigen::Matrix3d T1=Eigen::Matrix3d::Zero();
		T1(0,2)=-1;
		T1(1,0)=-1;
		T1(2,1)=1;

		// Bone default transformation 
		Eigen::Matrix3d T2=joint.getRotMatDefault();

		// calculate bone rotation matrix
		Eigen::Matrix3d R;
		R=T4*T3.inverse()*T1.inverse()*T2;
		Rotation=T2*R.inverse();
	}
	*/


	bool updateFromLeapHand(Leap::Hand &hand, Q2::ArmA5 &arm){

		// get leap arm model
		Leap::Arm Leaparm = hand.arm();
		
		// get frame timestamp from leap frame
		arm.setTimestamp(hand.frame().timestamp());

		// Select Start joint
		Q2::JointVertex WristVertex=2;
		Q2::SkeletonJoint wrist=arm.getSkeleton().getGraph()[WristVertex];

		// Set wrist position global
		Eigen::Vector3d WristPositionGlobal;
		Eigen::Vector3d LeapPositionGlobal(Leaparm.wristPosition()[0], Leaparm.wristPosition()[1], Leaparm.wristPosition()[2]);
		LeapToQ2Coord(LeapPositionGlobal, WristPositionGlobal);

		// Set wrist orientation global
		Eigen::Matrix3d RotMatGlobal_wrist;
		LeapRotationArm(Leaparm, hand, wrist , RotMatGlobal_wrist);
		//LeapRotationHand(hand, Leaparm, wrist, RotMatGlobal_wrist);

		// Set wrist as start joint
		arm.setStartJoint(wrist, WristPositionGlobal, RotMatGlobal_wrist);
		//this->update();

		/*
		// compute the forearm length
		Eigen::Vector3d ElbowPositionGlobal;
		Eigen::Vector3d LeapPositionElbow(Leaparm.elbowPosition()[0], Leaparm.elbowPosition()[1], Leaparm.elbowPosition()[2]);
		LeapToQ2Coord(LeapPositionElbow, ElbowPositionGlobal);
		Eigen::Vector3d vl=WristPositionGlobal-ElbowPositionGlobal;
		double forearm_length=sqrt(vl[0]*vl[0]+vl[1]*vl[1]+vl[2]*vl[2]);
		Q2::Skeleton S=arm.getSkeleton();
		Q2::SkeletonGraph G=S.getGraph();
		Q2::JointVertex vbone=2;
		Q2::in_edge_iter ei;
		Q2::in_edge_iter ei_end;
		boost::tie(ei, ei_end) = boost::in_edges(vbone, G);
		Q2::SkeletonBone b=G[*ei];
		b.setBoneLength(forearm_length);
		G[*ei]=b;
		S.setGraph(G);
		arm.setSkeleton(S);
		*/

		
		/*
		Q2::JointVertex vertex=3;
		Q2::SkeletonJoint Middelhandjoint;
		arm.getMiddelhand(Middelhandjoint);		
		Q2::SkeletonJoint joint=arm.getSkeleton().getGraph()[Middelhandjoint.getVertex()];
		Eigen::Matrix3d RotMatGlobal_middelhand;
		LeapRotationHand(hand, Leaparm, joint , RotMatGlobal_middelhand);

		// calculate rotation between arm and hand
		Leap::FingerList fingers = hand.fingers();
		Leap::Matrix Mh1;
		for(Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++){
			switch((*fl).type())
			{
			case Leap::Finger::TYPE_MIDDLE :
				Mh1=(*fl).bone(Leap::Bone::TYPE_PROXIMAL).basis();
				break;
			default:
				break;
			}


		}
		*/

		// get middelhand orientation
		Leap::Matrix Mh=hand.basis();
		Leap::Matrix Ma=Leaparm.basis();
		Eigen::Matrix3d MhE;
		Eigen::Matrix3d MaE;
		LeapMatToEigenMat(Mh,MhE);
		LeapMatToEigenMat(Ma,MaE);

		Eigen::Matrix3d R_hand_arm=MhE*MaE.inverse();

		double alpha=0; double alpha_min=-PI/2; double alpha_max=PI/2;
		double beta=0; double beta_min=-PI/2; double beta_max=PI/2;
		double gamma=0; double gamma_min=-PI/2; double gamma_max=PI/2;
		bool bo=Q2::EulerFromMat(R_hand_arm, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);
		
		// set wrist angles
		Q2::JointVertex v=2;
		
		double phi_wrist_FE;
		double phi_wrist_AA;
		double phi_wrist_PS;
		
		if(arm.getIsRight()){
			phi_wrist_FE=alpha;
			phi_wrist_AA=-beta;
			phi_wrist_PS=-gamma;
		}else{
			phi_wrist_FE=-alpha;
			phi_wrist_AA=-beta;
			phi_wrist_PS=-gamma;
		}

		double score=(double)hand.confidence();
		bool u1=arm.update(v, 0, phi_wrist_FE, score);
		bool u2=arm.update(v, 1, phi_wrist_AA, score);
		bool u3=arm.update(v, 2, phi_wrist_PS, score);
		if(u1 && u2 && u3){
			arm.update();
		}
		//std::cout << alpha << std::endl;
		return true;
	};

	
	bool updateFromLeapArm(Leap::Hand &hand, Q2::Arm &arm){

		// get leap arm model
		Leap::Arm Leaparm = hand.arm();
		
		// get frame timestamp from leap frame
		arm.setTimestamp(hand.frame().timestamp());

		// Select Start joint
		Q2::JointVertex WristVertex=2;
		Q2::SkeletonJoint wrist=arm.getSkeleton().getGraph()[WristVertex];

		// Set wrist position global
		Eigen::Vector3d WristPositionGlobal;
		Eigen::Vector3d LeapPositionGlobal(Leaparm.wristPosition()[0], Leaparm.wristPosition()[1], Leaparm.wristPosition()[2]);
		LeapToQ2Coord(LeapPositionGlobal, WristPositionGlobal);

		// Set wrist orientation global
		Eigen::Matrix3d RotMatGlobal_wrist;
		LeapRotationArm(Leaparm, hand, wrist , RotMatGlobal_wrist);
		//LeapRotationHand(hand, Leaparm, wrist, RotMatGlobal_wrist);

		// Set wrist as start joint
		arm.setStartJoint(wrist, WristPositionGlobal, RotMatGlobal_wrist);

		if(!arm.getSkeleton().getMapping() ){
			arm.update();
		}


		//std::cout << alpha << std::endl;
		return true;

	};
	

	bool updateFromLeapFinger(Leap::Hand &hand, Leap::Finger finger_leap, Q2::Finger &finger){

		

		if(finger_leap.type()==Leap::Finger::TYPE_THUMB){
			Leap::Matrix M1;
			Leap::Matrix M2;
			Eigen::Matrix3d M1E;
			Eigen::Matrix3d M2E;
			Eigen::Matrix3d R_bone;
			double alpha=0; double alpha_min=-PI/2; double alpha_max=PI/2;
			double beta=0; double beta_min=-PI/2; double beta_max=PI/2;
			double gamma=0; double gamma_min=-PI/2; double gamma_max=PI/2;
			double score=(double)hand.confidence();

			Q2::JointVertex v;

			// Select Start joint
			Q2::JointVertex MCPVertex=1;
			Q2::SkeletonJoint MCP=finger.getSkeleton().getGraph()[MCPVertex];

			// Set MCP position global
			Eigen::Vector3d MCPPositionGlobal;
			Eigen::Vector3d LeapPositionGlobal(finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[0], finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[1], finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[2]);
			LeapToQ2Coord(LeapPositionGlobal, MCPPositionGlobal);

			// Set MCP orientation global
			Eigen::Matrix3d RotMatGlobal_MCP;
			Leap::Vector PPCenter = finger_leap.jointPosition(Leap::Finger::JOINT_MCP) + (finger_leap.jointPosition(Leap::Finger::JOINT_PIP) - finger_leap.jointPosition(Leap::Finger::JOINT_MCP)) * .5;
			LeapRotationBone(finger_leap.bone(Leap::Bone::TYPE_METACARPAL), MCP , RotMatGlobal_MCP, finger.isright());

			// Set MCP as start joint
			finger.setStartJoint(MCPVertex, MCPPositionGlobal, RotMatGlobal_MCP);

			// calculate rotation between MC and PP
			M1=finger_leap.bone(Leap::Bone::TYPE_METACARPAL).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_PROXIMAL).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			// set MCP angles
			
			double phi_MCP_AA;
			double phi_MCP_FE;
			if(hand.isRight()){
				phi_MCP_AA=-alpha;
				phi_MCP_FE=beta;
			}else{
				phi_MCP_AA=-alpha;
				phi_MCP_FE=-beta;
			}
			
			

			v=1;
			bool b_MCP_AA=finger.update(v, 0, phi_MCP_AA, score);
			bool b_MCP_FE=finger.update(v, 1, phi_MCP_FE, score);

			//std::cout << "alpha: " << alpha << "     beta: " << beta << std::endl;


			// calculate rotation between PP and MP
			M1=finger_leap.bone(Leap::Bone::TYPE_PROXIMAL).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_INTERMEDIATE).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			alpha=0; alpha_min=-PI; alpha_max=PI/2;
			beta=0; beta_min=-PI/2; beta_max=PI/2;
			gamma=0; gamma_min=-PI/2; gamma_max=PI/2;
			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			// set CMC angles
			v=2;
			double phi_CMC_AA;
			double phi_CMC_FE;
			if(hand.isRight()){
				phi_CMC_AA=beta;
				phi_CMC_FE=alpha;
			}else{
				phi_CMC_AA=beta;
				phi_CMC_FE=alpha;
			}
			
			
			bool b_CMC_AA=finger.update(v, 0, phi_CMC_AA, score);		// Leap does not provide this DOF -> phi_CMC_FE=0!
			bool b_CMC_FE=finger.update(v, 1, phi_CMC_FE, score);		

			// std::cout << "alpha1: " << alpha << std::endl;

			// calculate rotation between MP and DP
			M1=finger_leap.bone(Leap::Bone::TYPE_INTERMEDIATE).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_DISTAL).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			alpha=0; alpha_min=-PI; alpha_max=PI/2;
			beta=0; beta_min=-PI/2; beta_max=PI/2;
			gamma=0; gamma_min=-PI/2; gamma_max=PI/2;
			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			// set IP angles
			v=3;
			double phi_IP_AA;
			if(hand.isRight()){
				phi_IP_AA=alpha;
			}else{
				phi_IP_AA=alpha;
			}
			bool b_IP=finger.update(v, 0, phi_IP_AA, score);

			//std::cout << "phi_IP_AA: " << phi_IP_AA << std::endl;
			//std::cout << "phi_CMC_AA: " << phi_CMC_AA << "     phi_CMC_FE: " << phi_CMC_FE << "   phi_IP_AA: " << phi_IP_AA << std::endl;
			
			// update finger
			//finger.update();
			if(b_MCP_FE && b_MCP_AA && b_CMC_AA && b_CMC_FE && b_IP && !finger.getSkeleton().getMapping()){
			//if(b_MCP_FE && b_MCP_AA && b_CMC_AA && b_CMC_FE && b_IP){
				finger.update();
			}else{
				int q=1;
			}

		}else{

			clock_t begin = clock();

			Leap::Matrix M1;
			Leap::Matrix M2;
			Eigen::Matrix3d M1E;
			Eigen::Matrix3d M2E;
			Eigen::Matrix3d R_bone;
			double alpha=0; double alpha_min=-PI/2; double alpha_max=PI/2;
			double beta=0; double beta_min=-PI/2; double beta_max=PI/2;
			double gamma=0; double gamma_min=-PI/2; double gamma_max=PI/2;
			double score=(double)hand.confidence();

			Q2::JointVertex v;

			// Select Start joint
			Q2::JointVertex MCPVertex=1;
			Q2::SkeletonJoint MCP=finger.getSkeleton().getGraph()[MCPVertex];

			// Set MCP position global
			Eigen::Vector3d MCPPositionGlobal;
			Eigen::Vector3d LeapPositionGlobal(finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[0], finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[1], finger_leap.jointPosition(Leap::Finger::JOINT_MCP)[2]);
			LeapToQ2Coord(LeapPositionGlobal, MCPPositionGlobal);

			// Set MCP orientation global
			Eigen::Matrix3d RotMatGlobal_MCP;
			Leap::Vector PPCenter = finger_leap.jointPosition(Leap::Finger::JOINT_MCP) + (finger_leap.jointPosition(Leap::Finger::JOINT_PIP) - finger_leap.jointPosition(Leap::Finger::JOINT_MCP)) * .5;
			LeapRotationBone(finger_leap.bone(Leap::Bone::TYPE_METACARPAL), MCP , RotMatGlobal_MCP,  finger.isright());

			// Set MCP as start joint
			finger.setStartJoint(MCPVertex, MCPPositionGlobal, RotMatGlobal_MCP);

			// calculate rotation between MC and PP
			M1=finger_leap.bone(Leap::Bone::TYPE_METACARPAL).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_PROXIMAL).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			// set MCP angles
			v=1;
			double phi_MCP_AA;
			double phi_MCP_FE;
			
			if(finger.isright()){
				phi_MCP_AA=-beta;
				phi_MCP_FE=-alpha;
			}else{
				phi_MCP_AA=-beta;
				phi_MCP_FE=alpha;
			}

			//std::cout << "phi_MCP_AA: " << phi_MCP_AA << "     phi_MCP_FE: " << phi_MCP_FE << std::endl;

			bool b_MCP_AA=finger.update(v, 0, phi_MCP_AA, score);
			bool b_MCP_FE=finger.update(v, 1, phi_MCP_FE, score);
			
			
			
			// calculate rotation between PP and MP
			M1=finger_leap.bone(Leap::Bone::TYPE_PROXIMAL).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_INTERMEDIATE).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			alpha=0; alpha_min=-PI; alpha_max=PI/2;
			beta=0; beta_min=-PI/2; beta_max=PI/2;
			gamma=0; gamma_min=-PI/2; gamma_max=PI/2;
			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			// set PIP angles
			v=2;
			double phi_PIP_FE;
			if(finger.isright()){
				phi_PIP_FE=-alpha;
			}else{
				phi_PIP_FE=alpha;
			}

			bool b_PIP=finger.update(v, 0, phi_PIP_FE, score);

			

			// calculate rotation between MP and DP
			M1=finger_leap.bone(Leap::Bone::TYPE_INTERMEDIATE).basis();
			M2=finger_leap.bone(Leap::Bone::TYPE_DISTAL).basis();
			LeapMatToEigenMat(M1,M1E);
			LeapMatToEigenMat(M2,M2E);
			R_bone=M2E*M1E.inverse();

			alpha=0; alpha_min=-PI; alpha_max=PI/2;
			beta=0; beta_min=-PI/2; beta_max=PI/2;
			gamma=0; gamma_min=-PI/2; gamma_max=PI/2;
			Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

			double elapsed_secs1 = double(clock() - begin) / CLOCKS_PER_SEC;

			// set DIP angles
			v=3;
			double phi_DIP_FE;
			if(finger.isright()){
				phi_DIP_FE=-alpha;
			}else{
				phi_DIP_FE=alpha;
			}

			bool b_DIP=finger.update(v, 0, phi_DIP_FE, score);

			double elapsed_secs2 = double(clock() - begin) / CLOCKS_PER_SEC;

			// update finger
			
			if( b_MCP_FE && b_MCP_AA && b_PIP && b_DIP && !finger.getSkeleton().getMapping() ){
				finger.update();
			}else{
				int q=1;
			}
		}
		
		return true;
	};
	

	bool updateWrist(Q2::HandArm &handarm, Leap::Hand &hand, Leap::Finger finger_leap, bool right){

		// calculate rotation between underarm and MC
		double alpha=0; double alpha_min=-PI/2; double alpha_max=PI/2;
		double beta=0; double beta_min=-PI/2; double beta_max=PI/2;
		double gamma=0; double gamma_min=-PI/2; double gamma_max=PI/2;

		Leap::Arm Leaparm = hand.arm();

		Eigen::Matrix3d M1E;
		Eigen::Matrix3d M2E;
		Eigen::Matrix3d R_bone;
		Leap::Matrix M1=Leaparm.basis();
		Leap::Matrix M2=finger_leap.bone(Leap::Bone::TYPE_METACARPAL).basis();
		LeapMatToEigenMat(M1,M1E);
		LeapMatToEigenMat(M2,M2E);
		R_bone=M2E*M1E.inverse();

		Q2::EulerFromMat(R_bone, alpha, alpha_min, alpha_max, beta, beta_min, beta_max, gamma, gamma_min, gamma_max);

		// wrist angles
		
		double phi_CMC_FE;
		double phi_CMC_AA;
		double phi_CMC_PS;
		if(right){
			phi_CMC_FE=alpha;
			phi_CMC_AA=-beta;
			phi_CMC_PS=gamma;
		}else{
			phi_CMC_FE=-alpha;
			phi_CMC_AA=-beta;
			phi_CMC_PS=gamma;
		}
		
		double score=(double)hand.confidence();
		Q2::JointVertex v=2;

		
		bool b_MCP_FE=handarm.update(v, 0, phi_CMC_FE, score);
		bool b_MCP_AA=handarm.update(v, 1, phi_CMC_AA, score);
		bool b_MCP_PS=handarm.update(v, 2, phi_CMC_PS, score);
		bool r = b_MCP_FE && b_MCP_AA && b_MCP_PS;
		
		//std::cout << "phi_CMC_FE: " << phi_CMC_FE << "     phi_CMC_AA: " << phi_CMC_AA << "     phi_CMC_PS: " << phi_CMC_PS << std::endl;

		return r;
	};

	bool updateFromLeapHandArm(Leap::Hand &hand, Q2::HandArm &handarm, int64_t timestamp){


		// update arm
		LeapToQ2::updateFromLeapArm(hand, handarm.getArm());
		handarm.setTimestamp(timestamp);
		//update fingers
		Leap::FingerList fingers = hand.fingers();
		for(Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++){
			switch((*fl).type())
				{
				case Leap::Finger::TYPE_PINKY :
					LeapToQ2::updateFromLeapFinger(hand, (*fl), handarm.getPinky());
					break;
				case Leap::Finger::TYPE_RING :
					LeapToQ2::updateFromLeapFinger(hand, (*fl), handarm.getRing());
					break;
				case Leap::Finger::TYPE_MIDDLE :
					LeapToQ2::updateFromLeapFinger(hand, (*fl), handarm.getMiddle());
					LeapToQ2::updateWrist(handarm, hand, (*fl), handarm.getIsRight());
					break;
				case Leap::Finger::TYPE_INDEX :
					LeapToQ2::updateFromLeapFinger(hand, (*fl), handarm.getIndex());
					break;
				case Leap::Finger::TYPE_THUMB :
					LeapToQ2::updateFromLeapFinger(hand, (*fl), handarm.getThumb());
					break;
				default:
					break;
				}
		}

		

		Q2::JointVertex v=2;
		handarm.setStartJoint(v, handarm.getArm().getWrist().getPositionGlobal(), handarm.getArm().getWrist().getRotMatGlobal());
		handarm.update();

		return true;
	};


	void LeapMatToEigenMat(Leap::Matrix &LeapMat, Eigen::Matrix3d &EigenMat){
		Leap::Vector Vx=LeapMat.xBasis;
		Leap::Vector Vy=LeapMat.yBasis;
		Leap::Vector Vz=LeapMat.zBasis;
		EigenMat(0,0)=Vx[0];
		EigenMat(0,1)=Vx[1];
		EigenMat(0,2)=Vx[2];
		EigenMat(1,0)=Vy[0];
		EigenMat(1,1)=Vy[1];
		EigenMat(1,2)=Vy[2];
		EigenMat(2,0)=Vz[0];
		EigenMat(2,1)=Vz[1];
		EigenMat(2,2)=Vz[2];
	};

	
	void LeapToQ2Coord(Eigen::Vector3d &leap_coord, Eigen::Vector3d &q2_coord){
		leap_coord=leap_coord/1000.0;
		double x_leap=0.3; // distance of the leap motion controller in global x
		double z_leap=0.01; // distance of the leap motion controller in global z
		Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
		Eigen::Vector4d v_leap(leap_coord[0],leap_coord[1],leap_coord[2],1);
		Eigen::Vector4d v_q2;
		T(0,2)=-1;
		T(0,3)=x_leap;
		T(1,0)=-1;
		T(2,1)=1;
		T(3,3)=1;
		T(2,3)=z_leap;
		v_q2=T*v_leap;
		q2_coord[0]=v_q2[0];
		q2_coord[1]=v_q2[1];
		q2_coord[2]=v_q2[2];
	};

	void EigenMatToLeapMat(Leap::Matrix &LeapMat, Eigen::Matrix3d &EigenMat){
		Leap::Vector xBasis = Leap::Vector((float)EigenMat(0,0), (float)EigenMat(1,0), (float)EigenMat(2,0));
		Leap::Vector yBasis = Leap::Vector((float)EigenMat(0,1), (float)EigenMat(1,1), (float)EigenMat(2,1));
		Leap::Vector zBasis = Leap::Vector((float)EigenMat(0,2), (float)EigenMat(1,2), (float)EigenMat(2,2));
		LeapMat = Leap::Matrix(xBasis, yBasis, zBasis);
	};

}