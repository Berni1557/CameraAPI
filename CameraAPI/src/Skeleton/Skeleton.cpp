/**
@file Skeleton.cpp

@author Bernhard Föllmer

Interface for hand tracking between subproject Q2 and subproject A5
*/

#include <Skeleton.h>
//#include "GLWidget.h"

namespace Q2
{
	#ifndef PI
	#define PI    3.14159265358979323846f
	#endif

	bool EulerFromMat(Eigen::Matrix3d &R, double &alpha, double alpha_min,  double alpha_max, double &beta, double beta_min,  double beta_max, double &gamma, double gamma_min,  double gamma_max){

		
		// compute rotation angles alpha, beta, gamma
		double beta1; double beta2;
		if(R(2,1)!=1 && R(2,1)!=-1){
			// calculate beta
			beta1=-asin(R(2,0));
			beta2=PI-beta1;
			if(beta1>beta_min && beta1<beta_max){
				if(beta2>beta_min && beta2<beta_max){
					return false;
				}else{
					beta=beta1;
				}
			}else{
				if(beta2>beta_min && beta2<beta_max){
					beta=beta2;
				}else{
					return false;
				}
			}

			// calculate alpha
			alpha=atan2(R(2,1)/cos(beta),R(2,2)/cos(beta));
			
			// calculate gamma
			gamma=atan2(R(1,0)/cos(beta),R(0,0)/cos(beta));
			return true;
		}else{
			return false;
		}
		/*
		alpha=atan2(R(2,1),R(2,2));
		double s=sqrt( R(2,1)*R(2,1) + R(2,2)*R(2,2) );
		beta=atan2(-R(2,0), s);
		gamma=atan2(R(1,0),R(0,0));
		return true;
		*/
	};

	/** @brief Transform the euler angle pitch (rotation about x-axis), yaw (rotation about y-axis), roll (rotation about z-axis) into rotation matrx
	*/
	Eigen::Matrix3d euler( const double pitch,const double yaw,const double roll){
		Eigen::Matrix3d Rx = Eigen::Matrix3d::Identity();
		Rx(1,1)=cos(pitch);
		Rx(1,2)=-sin(pitch);
		Rx(2,1)=sin(pitch);
		Rx(2,2)=cos(pitch);

		Eigen::Matrix3d Ry = Eigen::Matrix3d::Identity();
		Ry(0,0)=cos(yaw);
		Ry(0,2)=sin(yaw);
		Ry(2,0)=-sin(yaw);
		Ry(2,2)=cos(yaw);

		Eigen::Matrix3d Rz = Eigen::Matrix3d::Identity();
		Rz(0,0)=cos(roll);
		Rz(0,1)=-sin(roll);
		Rz(1,0)=sin(roll);
		Rz(1,1)=cos(roll);
		return Rx*Ry*Rz;
	};

	/** @brief Class for DH parameter
	*/
	DHParameter::DHParameter()
	{
	};

	/** @brief Concstructor for DHparameter
		@param phi is the angle phi of the DH parameter
	*/
	DHParameter::DHParameter(double phi,double d,double alpha,double a, std::pair <double,double> MinMaxPhi)
	{
		this->phi=phi;
		this->phi_t=0;
		this->d=d;
		this->alpha=alpha;
		this->a=a;
		this->MinMaxPhi=MinMaxPhi;
	};

	bool DHParameter::setPhi_t(double phi_t){
		if(phi_t>this->MinMaxPhi.first && phi_t<this->MinMaxPhi.second){
			this->phi_t=phi_t;
			return true;
		}else{
			return false;
		}
	};

	double DHParameter::getPhi(){
		return phi;
	};

	double DHParameter::getPhi_t(){
		return phi_t;
	};

	double DHParameter::getD(){
		return d;
	};

	Eigen::Matrix4d DHParameter::getRotz(){
		Eigen::Matrix4d Rotz = Eigen::Matrix4d::Zero();
		Rotz(0,0)=std::cos(phi+phi_t);
		Rotz(0,1)=-std::sin(phi+phi_t);
		Rotz(1,0)=std::sin(phi+phi_t);
		Rotz(1,1)=std::cos(phi+phi_t);
		Rotz(2,2)=1;
		Rotz(3,3)=1;
		return Rotz;
	};

	Eigen::Matrix4d DHParameter::getRotz(double phi_n){
		Eigen::Matrix4d Rotz = Eigen::Matrix4d::Zero();
		Rotz(0,0)=std::cos(phi_n+phi_t);
		Rotz(0,1)=-std::sin(phi_n+phi_t);
		Rotz(1,0)=std::sin(phi_n+phi_t);
		Rotz(1,1)=std::cos(phi_n+phi_t);
		Rotz(2,2)=1;
		Rotz(3,3)=1;
		return Rotz;
	};

	Eigen::Matrix4d DHParameter::getTransz(){
		Eigen::Matrix4d Transz = Eigen::Matrix4d::Zero();
		Transz(0,0)=1;
		Transz(1,1)=1;
		Transz(2,2)=1;
		Transz(3,3)=1;
		Transz(2,3)=d;
		return Transz;
	};

	Eigen::Matrix4d DHParameter::getTransz(double d_n){
		Eigen::Matrix4d Transz = Eigen::Matrix4d::Zero();
		Transz(0,0)=1;
		Transz(1,1)=1;
		Transz(2,2)=1;
		Transz(3,3)=1;
		Transz(2,3)=d_n;
		return Transz;
	};

	Eigen::Matrix4d DHParameter::getRotx(){
		Eigen::Matrix4d Rotx = Eigen::Matrix4d::Zero();
		Rotx(0,0)=1;
		Rotx(1,1)=std::cos(alpha);
		Rotx(1,2)=-std::sin(alpha);
		Rotx(2,1)=std::sin(alpha);
		Rotx(2,2)=std::cos(alpha);
		Rotx(3,3)=1;
		return Rotx;
	};

	Eigen::Matrix4d DHParameter::getTransx(){
		Eigen::Matrix4d Transx = Eigen::Matrix4d::Zero();
		Transx(0,0)=1;
		Transx(1,1)=1;
		Transx(2,2)=1;
		Transx(3,3)=1;
		Transx(0,3)=a;
		return Transx;
	};

	Eigen::Matrix4d DHParameter::getT(){
		return T;
	};

	std::pair <double,double> DHParameter::getMinMaxPhi(){
		return this->MinMaxPhi;
	};

	DHParameter::~DHParameter()
	{
	};

	JointDOF::JointDOF()
	{
		this->score=0;
	};

	JointDOF::JointDOF(DHParameter DH, movementType mtype)
	{
		this->DH=DH;
		this->mtype=mtype;
		this->score=0;
	};

	bool JointDOF::updatePhi_t(double phi_t){
		return DH.setPhi_t(phi_t);
	};

	void JointDOF::setDHParameter(DHParameter DH){
		this->DH=DH;
	};


	void JointDOF::setPositionGlobal(Eigen::Vector3d PositionGlobal){
		this->PositionGlobal=PositionGlobal;
	};

	void JointDOF::setQuaternionGlobal(Eigen::Quaternion<double,Eigen::DontAlign> q){
		this->q=q;
	};

	void JointDOF::setMtype(movementType mtype){
		this->mtype=mtype;
	};

	void JointDOF::setScore(double score){
		this->score=score;
	};

	DHParameter& JointDOF::getDHParameter(){
		return DH;
	};

	void JointDOF::setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal){
		this->RotMatGlobal=RotMatGlobal;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> JointDOF::getRotMatGlobal(){
		return RotMatGlobal;
	};

	Eigen::Vector3d JointDOF::getPositionGlobal(){
		return PositionGlobal;
	};

	Eigen::Quaternion<double,Eigen::DontAlign> JointDOF::getQuaternionGlobal(){
		return q;
	};

	movementType JointDOF::getMtype(){
		return mtype;
	};

	double JointDOF::getScore(){
		return score;
	};

	JointDOF::~JointDOF()
	{
	};

	SkeletonJoint::SkeletonJoint(){
		PositionGlobal=Eigen::Vector3d(0,0,0);
		q=Eigen::Quaternion<double,Eigen::DontAlign>(1,0,0,0);
	};

	SkeletonJoint::SkeletonJoint(std::vector <JointDOF> DOFvec, skeletonJointType type, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion){
		this->DOFvec=DOFvec;
		this->type=type;
		PositionGlobal=Eigen::Vector3d(0,0,0);
		q=Eigen::Quaternion<double,Eigen::DontAlign>(1,0,0,0);
		this->RotMatDefault=RotMatDefault;
		this->RotMatDefaultToUnion=RotMatDefaultToUnion;
	};

	bool SkeletonJoint::updatePhi(int DOF, double phi_t, double score){
		bool inrange;
		if(DOF < (int)DOFvec.size()){
			inrange=DOFvec[DOF].updatePhi_t(phi_t);
			if(inrange){
				DOFvec[DOF].setScore(score);
			}else{
				DOFvec[DOF].setScore(0.0);
			}
			return inrange;
		}else{
			return false;
		}
	};

	bool SkeletonJoint::update(SkeletonGraph &graph){

		clock_t begin = clock();

		update_Position_Orientation();
		double elapsed_secs1 = double(clock() - begin) / CLOCKS_PER_SEC;
		update_out_joints(graph);
		double elapsed_secs2 = double(clock() - begin) / CLOCKS_PER_SEC;
		update_in_joints(graph);
		double elapsed_secs3 = double(clock() - begin) / CLOCKS_PER_SEC;
		graph[vertex]=(*this);

		double elapsed_secs4 = double(clock() - begin) / CLOCKS_PER_SEC;
		return true;
	};

	bool SkeletonJoint::update(SkeletonGraph &graph, SkeletonJoint &origen, edgeDirection edgeDir){
		// update joint position and orientation
		int v=(int)this->vertex;
		update_Position_Orientation(origen, edgeDir);

		// update bone position and orientation
		if(edgeDir==Edge_IN){
			BoneEdge b=boost::edge(origen.vertex,this->vertex,graph).first;
			graph[b].update(*this);
		}
		else{
			BoneEdge b=boost::edge(this->vertex,origen.vertex,graph).first;
			graph[b].update(origen);
		}
		
		// update joints from out edges
		update_out_joints(graph, origen);

		// update joints from in edges
		update_in_joints(graph, origen);

		graph[vertex]=(*this);
		return true;
	};

	void SkeletonJoint::setPositionGlobal(Eigen::Vector3d PositionGlobal){
		this->PositionGlobal=PositionGlobal;
	};

	void SkeletonJoint::setQGlobal(Eigen::Quaternion<double,Eigen::DontAlign> q){
		this->qGlobal=q;
	};

	void SkeletonJoint::setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal){ 
		this->RotMatGlobal=RotMatGlobal;
	};

	void SkeletonJoint::setRotMatDefault(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault){ 
		this->RotMatDefault=RotMatDefault;
	};

	void SkeletonJoint::setRotMatDefaultToUnion(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion){ 
		this->RotMatDefaultToUnion=RotMatDefaultToUnion;
	};

	void SkeletonJoint::setShape(Sphere JointShape){
		this->JointShape=JointShape;
	};

	void SkeletonJoint::setDOFvec(std::vector <JointDOF> DOFvec){
		this->DOFvec=DOFvec;
	};

	void SkeletonJoint::setVertex(JointVertex vertex){
		this->vertex=vertex;
	};

	JointVertex SkeletonJoint::getVertex(){
		return vertex;
	};

	bool SkeletonJoint::update_in_joints(SkeletonGraph &graph, SkeletonJoint &origen){

		in_edge_iter ei;
		in_edge_iter ei_end;
		for (boost::tie(ei, ei_end) = boost::in_edges(vertex, graph); ei != ei_end; ++ei){
			if(graph[boost::source(*ei,graph)].getVertex()!=origen.getVertex()){
				graph[boost::source(*ei,graph)].update(graph, *this, Edge_OUT);
			}
		}
		return true;
	};

	bool SkeletonJoint::update_in_joints(SkeletonGraph &graph){
		in_edge_iter ei;
		in_edge_iter ei_end;
		
		clock_t begin = clock();

		for (boost::tie(ei, ei_end) = boost::in_edges(vertex, graph); ei != ei_end; ++ei){
			graph[boost::source(*ei,graph)].update(graph, *this, Edge_OUT);
		}
		double elapsed_secs1 = double(clock() - begin) / CLOCKS_PER_SEC;
		return true;
	};


	bool SkeletonJoint::update_out_joints(SkeletonGraph &graph, SkeletonJoint &origen){
		out_edge_iter ei;
		out_edge_iter ei_end;

		for (boost::tie(ei, ei_end) = boost::out_edges(vertex, graph); ei != ei_end; ++ei){
			if(graph[boost::target(*ei,graph)].getVertex()!=origen.getVertex()){
				graph[boost::target(*ei,graph)].update(graph, *this, Edge_IN);
				graph[*ei].update(graph[boost::target(*ei,graph)]); // update bone b based on joint jo
			}
		}
		
		//std::pair<out_edge_iter, out_edge_iter> p1 = boost::out_edges((*StartJoint).getVertex(), Graph);
		return true;
	};

	bool SkeletonJoint::update_out_joints(SkeletonGraph &graph){
		out_edge_iter ei;
		out_edge_iter ei_end;
		for (boost::tie(ei, ei_end) = boost::out_edges(vertex, graph); ei != ei_end; ++ei){
			graph[boost::target(*ei,graph)].update(graph, *this, Edge_IN);
		}
		return true;
	};

	
	bool SkeletonJoint::update_Position_Orientation(){
		Eigen::Quaternion<double> qn1;
		Eigen::Quaternion<double> qn2;
		Eigen::Quaternion<double> qT;
		Eigen::Matrix4d T=Eigen::Matrix4d::Zero();
		Eigen::Matrix3d N=Eigen::Matrix3d::Zero();
		Eigen::Matrix4d Tn=Eigen::Matrix4d::Zero();
		Eigen::Vector3d vn1(0,0,0);
		Eigen::Vector3d vn2(0,0,0);
		Eigen::Vector3d vT(0,0,0);
		this;
		Eigen::Matrix3d RotMatTemp;
		Eigen::Matrix3d RotMatT;
		double phi_n;
		double d_n;
		if(DOFvec.size()>0){
			for (int i=0;i<(int)DOFvec.size();i++){
				if(i==0){

					// set JointDOF parameter
					DOFvec[i].setRotMatGlobal(RotMatGlobal);
					DOFvec[i].setQuaternionGlobal(Eigen::Quaternion<double>(RotMatGlobal));
					DOFvec[i].setPositionGlobal(PositionGlobal);
					qn1=RotMatGlobal;
					qGlobal=qn1;
					vn1=PositionGlobal;
				}
				else{
					// create DH-Matrix T
					Eigen::Matrix4d Tx=DOFvec[i].getDHParameter().getTransx();
					Eigen::Matrix4d Rx=DOFvec[i].getDHParameter().getRotx();
					d_n=DOFvec[i].getDHParameter().getD();
					Eigen::Matrix4d Tz=DOFvec[i-1].getDHParameter().getTransz(d_n);
					phi_n=DOFvec[i].getDHParameter().getPhi();
					Eigen::Matrix4d Rz=DOFvec[i-1].getDHParameter().getRotz(phi_n);
					T=Rz*Tz*Rx*Tx;
					
					// extract quaternion qT and vector vT from T
					RotMatT=T.block(0,0,3,3);
					qT=RotMatT;
					vT=T.block(0,3,3,1);

					// rotate qn by qT
					qn1=DOFvec[i-1].getQuaternionGlobal();
					qn2=qn1*qT;

					// set JointDOF parameter
					DOFvec[i].setRotMatGlobal(qn2.toRotationMatrix());
					DOFvec[i].setQuaternionGlobal(qn2);
					DOFvec[i].setPositionGlobal(PositionGlobal);
				}
			}
		}
		return true;
	};
	

	bool SkeletonJoint::update_Position_Orientation(SkeletonJoint &origen, edgeDirection edgeDir){
		// this->vertex=0, origen.vertex=1
		
		int DOFsize_origen;
		int DOFsize;
		Eigen::Quaternion<double> qn1;
		Eigen::Quaternion<double> qn2;
		Eigen::Quaternion<double> qT;
		Eigen::Matrix4d T=Eigen::Matrix4d::Zero();
		Eigen::Matrix4d Tn=Eigen::Matrix4d::Zero();
		Eigen::Vector3d vn1(0,0,0);
		Eigen::Vector3d vn2(0,0,0);
		Eigen::Vector3d vT(0,0,0);
		Eigen::Vector3d vTx(0,0,0);
		Eigen::Vector3d vTGlobal(0,0,0);
		Eigen::Matrix3d RotMatTemp;
		Eigen::Matrix3d RotMatT;
		Eigen::Matrix3d N;
		double phi_n;
		double d_n;
		int v=(int)this->vertex;
		// iteration over all jointDOF from joints
		if(DOFvec.size()>0){
			if(edgeDir==Edge_IN){
				for (int i=0;i<(int)DOFvec.size();i++){
					if(i==0){
						DOFsize_origen=(int)origen.getDOFvec().size();
						DOFsize=(int)DOFvec.size();
						Eigen::Matrix4d Tx=DOFvec[i].getDHParameter().getTransx();
						Eigen::Matrix4d Rx=DOFvec[i].getDHParameter().getRotx();
						d_n=DOFvec[i].getDHParameter().getD();
						Eigen::Matrix4d Tz=origen.getDOFvec()[DOFsize_origen-1].getDHParameter().getTransz(d_n);
						phi_n=DOFvec[i].getDHParameter().getPhi();
						Eigen::Matrix4d Rz=origen.getDOFvec()[DOFsize_origen-1].getDHParameter().getRotz(phi_n);
						T=Rz*Tz*Rx*Tx;

						// get quaternion and position from previous JointDOF
						qn1=origen.getDOFvec()[DOFsize_origen-1].getQuaternionGlobal();
						vn1=origen.getDOFvec()[DOFsize_origen-1].getPositionGlobal();	
						N=qn1.toRotationMatrix();

						// RotMatT quaternion qT and vector vT from T
						RotMatT=T.block(0,0,3,3);
						qT=RotMatT;
						vT=T.block(0,3,3,1);

						// rotate qn by qT
						qn2=qn1*qT;
						N=qn2.toRotationMatrix();
						//RotMatTemp=qn2.toRotationMatrix();

						// translation from vn to vn+vT
						vTGlobal=qn1*vT;
						vn2=vn1+vTGlobal;

						// set JointDOF parameter
						DOFvec[0].setRotMatGlobal(qn2.toRotationMatrix());
						DOFvec[0].setQuaternionGlobal(qn2);
						DOFvec[0].setPositionGlobal(vn2);

						// set joint position and orientation
						PositionGlobal=vn2;
						RotMatGlobal=qn2.toRotationMatrix();
						qGlobal=qn2;

					}else{

						// create DH-Matrix T
						Eigen::Matrix4d Tx=this->getDOFvec()[i].getDHParameter().getTransx();
						Eigen::Matrix4d Rx=this->getDOFvec()[i].getDHParameter().getRotx();
						d_n=DOFvec[i].getDHParameter().getD();
						Eigen::Matrix4d Tz=this->getDOFvec()[i-1].getDHParameter().getTransz(d_n);
						phi_n=DOFvec[i].getDHParameter().getPhi();
						Eigen::Matrix4d Rz=this->getDOFvec()[i-1].getDHParameter().getRotz(phi_n);
						T=Rz*Tz*Rx*Tx;

						// extract quaternion qT and vector vT from T
						RotMatT=T.block(0,0,3,3);
						qT=RotMatT;

						// rotate qn by qT
						qn1=DOFvec[i-1].getQuaternionGlobal();
						qn2=qn1*qT;

						// set JointDOF parameter
						DOFvec[i].setRotMatGlobal(qn2.toRotationMatrix());
						DOFvec[i].setQuaternionGlobal(qn2);
						DOFvec[i].setPositionGlobal(PositionGlobal);
					}
				}
			}else{
				for (int i=0;i<(int)DOFvec.size();i++){
					if(i==0){
						DOFsize_origen=(int)origen.getDOFvec().size();
						DOFsize=(int)DOFvec.size();
						Eigen::Matrix4d Tx=origen.getDOFvec()[0].getDHParameter().getTransx();
						Eigen::Matrix4d Rx=origen.getDOFvec()[0].getDHParameter().getRotx();
						d_n=origen.getDOFvec()[0].getDHParameter().getD();
						Eigen::Matrix4d Tz=DOFvec[DOFsize-1].getDHParameter().getTransz(d_n);
						phi_n=origen.getDOFvec()[0].getDHParameter().getPhi();
						Eigen::Matrix4d Rz=DOFvec[DOFsize-1].getDHParameter().getRotz(phi_n);
						T=Rz*Tz*Rx*Tx;
						

						// get quaternion and position from previous JointDOF
						qn1=origen.getDOFvec()[0].getQuaternionGlobal();
						vn1=origen.getDOFvec()[0].getPositionGlobal();	

						// extract quaternion qT and vector vT from T
						RotMatT=T.block(0,0,3,3);
						qT=RotMatT;
						vT=T.block(0,3,3,1);

						// rotate qn by qT inverse
						qn2=qn1*qT.inverse();

						// translation from vn to vn+vT
						vTGlobal=qn2*vT;
						vn2=vn1-vTGlobal;

						// set JointDOF parameter
						DOFvec[DOFsize-1].setRotMatGlobal(qn2.toRotationMatrix());
						DOFvec[DOFsize-1].setQuaternionGlobal(qn2);
						DOFvec[DOFsize-1].setPositionGlobal(vn2);

						// set joint position and orientation
						PositionGlobal=vn2;
						RotMatGlobal=qn2.toRotationMatrix();
						qGlobal=qn2;
					}else{

						// create DH-Matrix T
						DOFsize_origen=(int)origen.getDOFvec().size();
						DOFsize=(int)DOFvec.size();
						Eigen::Matrix4d Tx=DOFvec[DOFsize-i].getDHParameter().getTransx();
						Eigen::Matrix4d Rx=DOFvec[DOFsize-i].getDHParameter().getRotx();
						d_n=DOFvec[DOFsize-i].getDHParameter().getD();
						Eigen::Matrix4d Tz=DOFvec[DOFsize-i-1].getDHParameter().getTransz(d_n);
						phi_n=DOFvec[DOFsize-i].getDHParameter().getPhi();
						Eigen::Matrix4d Rz=DOFvec[DOFsize-i-1].getDHParameter().getRotz(phi_n);
						T=Rz*Tz*Rx*Tx;

						// get quaternion and position from previous JointDOF
						qn1=DOFvec[DOFsize-i].getQuaternionGlobal();
						vn1=DOFvec[DOFsize-i].getPositionGlobal();	

						// extract quaternion qT and vector vT from T
						RotMatT=T.block(0,0,3,3);
						qT=RotMatT;

						// rotate qn by qT inverse
						qn2=qn1*qT.inverse();

						// set JointDOF parameter
						DOFvec[DOFsize-i-1].setRotMatGlobal(qn2.toRotationMatrix());
						DOFvec[DOFsize-i-1].setQuaternionGlobal(qn2);
						DOFvec[DOFsize-i-1].setPositionGlobal(PositionGlobal);

					}
				}
			}
		}
		return true;
	};

	bool SkeletonJoint::getJointDOFbyMovementType(JointDOF &joint_out, movementType type){
		for(int i=0;i<(int)DOFvec.size();i++){
			if(DOFvec[i].getMtype()==type){
				joint_out=DOFvec[i];
				return true;
			}
		}
		return false;
	};

	
	bool SkeletonJoint::init(shapeType Stype){
		switch(Stype)
		{
			case sphere:
				{
					JointShape.init(0.001, &PositionGlobal, &getQuaternionUnion());
					return false;
				}
			case cylinder:
				{
					//Cylinder c;
					//c.init(boneWidth, boneLength, &PositionGlobal, &getQuaternionUnion()); 
					return true;
				}
			case cone:
				{
					return false;
				}
			default:
				{
					return false;
				}

		}
	};
	

	std::vector <JointDOF>& SkeletonJoint::getDOFvec(){
		return DOFvec;
	};

	Eigen::Vector3d  SkeletonJoint::getPositionGlobal(){
		return PositionGlobal;
	};

	Eigen::Quaternion<double,Eigen::DontAlign> SkeletonJoint::getQGlobal(){
		return qGlobal;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonJoint::getRotMatGlobal(){
		return RotMatGlobal;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonJoint::getRotMatDefault(){
		return RotMatDefault;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonJoint::getRotMatDefaultToUnion(){
		return RotMatDefaultToUnion;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonJoint::getRotMatUnion(){
		Eigen::Matrix<double,3,3,Eigen::DontAlign> M = this->RotMatGlobal * this->RotMatDefaultToUnion.inverse();
		return M;
	};

	Eigen::Quaternion<double,Eigen::DontAlign> SkeletonJoint::getQuaternionUnion(){
		Eigen::Matrix<double,3,3,Eigen::DontAlign> M = this->RotMatGlobal * this->RotMatDefaultToUnion.inverse();
		Eigen::Quaternion<double,Eigen::DontAlign> q;
		q=M;
		return q;
	};

	Sphere SkeletonJoint::getShape(){
		return JointShape;
	};

	SkeletonJoint::~SkeletonJoint(){
	};


	SkeletonBone::SkeletonBone()
	{
		this->boneLength=0;
		this->boneWidth=0;
	};


	SkeletonBone::SkeletonBone(double boneLength, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion)
	{
		this->boneLength=boneLength;
		this->boneWidth=0;
		this->RotMatDefault=RotMatDefault;
		this->RotMatDefaultToUnion=RotMatDefaultToUnion;
	};

	SkeletonBone::SkeletonBone(double boneLength, double boneWidth, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion)
	{
		this->boneLength=boneLength;
		this->boneWidth=boneWidth;
		this->RotMatDefault=RotMatDefault;
		this->RotMatDefaultToUnion=RotMatDefaultToUnion;
	};

	void SkeletonBone::setEdge(BoneEdge edge){
		this->edge=edge;
	};

	void SkeletonBone::setBoneLength(double boneLength){
		this->boneLength=boneLength;
	};

	void SkeletonBone::setShape(Cylinder BoneShape){
		this->BoneShape=BoneShape;
	};

	void SkeletonBone::setPositionGlobal(Eigen::Vector3d PositionGlobal){
		this->PositionGlobal=PositionGlobal;
	};

	void SkeletonBone::setQGlobal(Eigen::Quaternion<double,Eigen::DontAlign> qGlobal){
		this->qGlobal=qGlobal;
	};

	void SkeletonBone::setRotMatGlobal(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal){
		this->RotMatGlobal=RotMatGlobal;
	};

	void SkeletonBone::setRotMatDefault(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefault){
		this->RotMatDefault=RotMatDefault;
	};

	void SkeletonBone::setRotMatDefaultToUnion(Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatDefaultToUnion){
		this->RotMatDefaultToUnion=RotMatDefaultToUnion;
	};
	
	BoneEdge SkeletonBone::getEdge(){
		return edge;
	}

	Eigen::Vector3d SkeletonBone::getPositionGlobal(){
		return PositionGlobal;
	};

	Eigen::Quaternion<double,Eigen::DontAlign> SkeletonBone::getQGlobal(){
		return qGlobal;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonBone::getRotMatGlobal(){
		return RotMatGlobal;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonBone::getRotMatDefault(){
		return RotMatDefault;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonBone::getRotMatDefaultToUnion(){
		return RotMatDefaultToUnion;
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> SkeletonBone::getRotMatUnion(){
		Eigen::Matrix<double,3,3,Eigen::DontAlign> M = this->RotMatGlobal * this->RotMatDefaultToUnion.inverse();
		return M;
	};

	Eigen::Quaternion<double,Eigen::DontAlign> SkeletonBone::getQuaternionUnion(){
		Eigen::Matrix<double,3,3,Eigen::DontAlign> M = this->RotMatGlobal * this->RotMatDefaultToUnion.inverse();
		Eigen::Quaternion<double,Eigen::DontAlign> q;
		q=M;
		return q;
	};

	double SkeletonBone::getBoneLength(){
		return boneLength;
	};

	Cylinder SkeletonBone::getShape(){
		return BoneShape;
	};

	bool SkeletonBone::update(SkeletonJoint &joint){
		PositionGlobal=joint.getPositionGlobal();
		RotMatGlobal=joint.getRotMatGlobal();
		qGlobal=joint.getQGlobal();
		return true;
	};

	
	bool SkeletonBone::init(shapeType Stype){
		switch(Stype)
		{
			case sphere:
				{
					return false;
				}
			case cylinder:
				{
					BoneShape.init(0.01, boneLength, &PositionGlobal, &getQuaternionUnion()); 
					return true;
				}
			case cone:
				{
					return false;
				}
			default:
				{
					return false;
				}

		}
	};
	

	SkeletonBone::~SkeletonBone()
	{
	};

	Skeleton::Skeleton()
	{
		this->Mapping=false;
	}
		/*@brief empty destructor
		*/
	Skeleton::~Skeleton()
	{

	}


	void Skeleton::show()
	{

	};

	bool Skeleton::addJoint(SkeletonJoint &joint)
	{
		JointVertex v = boost::add_vertex(joint,Graph);
		joint=Graph[v];
		joint.setVertex(v);
		Graph[v]=joint;
		return true;
	};

	bool Skeleton::addJoint(SkeletonJoint &joint, JointVertex &joint_vertex, SkeletonBone bone, JointVertex &joint_vertex_out)
	{
		JointVertex v = boost::add_vertex(joint,Graph);
		std::pair<BoneEdge, bool> p = boost::add_edge(joint_vertex, v, bone, Graph);
		joint_vertex_out=v;
		return true;
	};

	bool Skeleton::addBone(SkeletonBone bone, SkeletonJoint &joint_start, SkeletonJoint &joint_end)
	{
		JointVertex joint_vertex_start;
		JointVertex joint_vertex_end;

		std::pair<vertex_iter, vertex_iter> vp;
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			if(*vp.first==joint_start.getVertex()){
				joint_vertex_start=*vp.first;
			}
			if(*vp.first==joint_end.getVertex()){
				joint_vertex_end=*vp.first;
			}
		}
		std::pair<BoneEdge, bool> p = boost::add_edge(joint_vertex_start, joint_vertex_end, bone, Graph);
		return true;
	};
	
	Eigen::Vector3d Skeleton::getGlobalPositionVertex(JointVertex joint_vertex){
		return Graph[joint_vertex].getPositionGlobal();
	};

	Eigen::Quaternion<double,Eigen::DontAlign> Skeleton::getQGlobalVertex(JointVertex joint_vertex){
		return Graph[joint_vertex].getQGlobal();
	};

	Eigen::Matrix<double,3,3,Eigen::DontAlign> Skeleton::getRotMatGlobalVertex(JointVertex joint_vertex){
		return Graph[joint_vertex].getRotMatGlobal();
	};
	
	
	Eigen::Vector3d Skeleton::getGlobalPositionEdge(JointVertex jv_start, JointVertex jv_end){
		Eigen::Vector3d v;
		Q2::BoneEdge be;
		Q2::SkeletonBone sb;
		bool b;
		boost::tie(be, b)=boost::edge(jv_start, jv_end, Graph);
		sb=Graph[be];
		v=sb.getPositionGlobal();
		return v;
	};

	
	Eigen::Quaternion<double,Eigen::DontAlign> Skeleton::getQGlobalEdge(JointVertex jv_start, JointVertex jv_end){
		Eigen::Vector3d v;
		Q2::BoneEdge be;
		Q2::SkeletonBone sb;
		bool b;
		boost::tie(be, b)=boost::edge(jv_start, jv_end, Graph);
		sb=Graph[be];
		return sb.getQGlobal();
	};
	
	Eigen::Matrix<double,3,3,Eigen::DontAlign> Skeleton::getRotMatGlobalEdge(JointVertex jv_start, JointVertex jv_end){
		Eigen::Vector3d v;
		Q2::BoneEdge be;
		Q2::SkeletonBone sb;
		bool b;
		boost::tie(be, b)=boost::edge(jv_start, jv_end, Graph);
		sb=Graph[be];
		return sb.getRotMatGlobal();
	};
	
	vertex_map Skeleton::getMap(){
		return this->VMap;
	};

	bool Skeleton::getMapping(){
		return this->Mapping;
	}

	void Skeleton::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Quaternion<double,Eigen::DontAlign> q)
	{
		Graph[joint.getVertex()].setPositionGlobal(PositionGlobal);
		Graph[joint.getVertex()].setQGlobal(q);
		Graph[joint.getVertex()].setRotMatGlobal(q.matrix());
		StartJoint=&joint;
	};
	
	void Skeleton::setStartJoint(SkeletonJoint &joint, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		if(Mapping){
			JointVertex v;
			v=VMap[joint.getVertex()];
			MappingSkeleton->getGraph()[v].setPositionGlobal(PositionGlobal);
			MappingSkeleton->getGraph()[v].setRotMatGlobal(RotMatGlobal);
			MappingSkeleton->getGraph()[v].setQGlobal(Eigen::Quaternion<double>(RotMatGlobal));
			
		}else{
			Graph[joint.getVertex()].setPositionGlobal(PositionGlobal);
			Graph[joint.getVertex()].setRotMatGlobal(RotMatGlobal);
			Graph[joint.getVertex()].setQGlobal(Eigen::Quaternion<double>(RotMatGlobal));
			StartJoint=&Graph[joint.getVertex()];
		}
	};

	void Skeleton::setStartJoint(JointVertex &vertex, Eigen::Vector3d PositionGlobal, Eigen::Matrix<double,3,3,Eigen::DontAlign> RotMatGlobal)
	{
		Graph[vertex].setPositionGlobal(PositionGlobal);
		Graph[vertex].setRotMatGlobal(RotMatGlobal);
		Graph[vertex].setQGlobal(Eigen::Quaternion<double>(RotMatGlobal));
		StartJoint=&Graph[vertex];
	};

	void Skeleton::setStartJoint(JointVertex &vertex){
		StartJoint=&Graph[vertex];
	};

	void Skeleton::setGraph(SkeletonGraph Graph){
		this->Graph=Graph;
	};

	void Skeleton::setMap(vertex_map map){
		this->VMap=map;
	};

	void Skeleton::setMapping(bool Mapping){
		this->Mapping=Mapping;
	};

	void Skeleton::setMappingSkeleton(Skeleton& MappingSkeleton){
		this->MappingSkeleton=&MappingSkeleton;
	};

	Skeleton* Skeleton::getMappingSkeleton(){
		return this->MappingSkeleton;
	};

	bool Skeleton::update(){	
		//(*StartJoint).update(Graph);

		if(Mapping){
			bool b1=(*StartJoint).update(Graph);
			//SkeletonGraph Gx;
			//Gx=MappingSkeleton->getGraph();
			//bool b2=MappingSkeleton->getStartJoint()->update(Gx);
			//MappingSkeleton->setGraph(Gx);

			bool b2=MappingSkeleton->getStartJoint()->update(MappingSkeleton->getGraph());

			return b1 && b2;
		}else{
			return (*StartJoint).update(Graph);
		}
	};

	SkeletonJoint* Skeleton::getStartJoint(){
		return StartJoint;
	};

	bool Skeleton::update(JointVertex vertex, int DOF, double phi_t, double score){
		clock_t begin = clock();
		if(Mapping){
			bool b1=Graph[vertex].updatePhi(DOF,phi_t, score);

			// !!!!!!!!! Copy of Graph takes almost 6 ms !!!!!!!!!
			/*
			SkeletonGraph Gx;
			Gx=MappingSkeleton->getGraph();
			JointVertex v=VMap[vertex];
			bool b2=Gx[v].updatePhi(DOF,phi_t, score);
			MappingSkeleton->setGraph(Gx);
			*/
			JointVertex v=VMap[vertex];
			bool b2=MappingSkeleton->getGraph()[v].updatePhi(DOF,phi_t, score);

			double elapsed_secs3 = double(clock() - begin) / CLOCKS_PER_SEC;

			return b1 && b2;
		}else{
			return Graph[vertex].updatePhi(DOF,phi_t, score);
		}
	};

	Eigen::VectorXd Skeleton::getState(){
		
		std::pair <double,double> MinMaxPhi;
		std::pair<vertex_iter, vertex_iter> vp;
		int num_elem=0;
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					num_elem++;
				}else{
					int q=1;
				}
			}
		}
		int k=0;
		
		Eigen::VectorXd V(num_elem);
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					V[k]=Graph[*vp.first].getDOFvec()[i].getDHParameter().getPhi_t();
					k++;
				}
				
			}
		}
		return V;
	};

	Eigen::VectorXd Skeleton::getMinState(){
		
		std::pair <double,double> MinMaxPhi;
		std::pair<vertex_iter, vertex_iter> vp;
		int num_elem=0;
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					num_elem++;
				}
			}
		}
		int k=0;
		
		Eigen::VectorXd V(num_elem);
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					V[k]=MinMaxPhi.first;
					k++;
				}
				
			}
		}
		return V;
	};

	Eigen::VectorXd Skeleton::getMaxState(){
		
		std::pair <double,double> MinMaxPhi;
		std::pair<vertex_iter, vertex_iter> vp;
		int num_elem=0;
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					num_elem++;
				}
			}
		}
		int k=0;
		
		Eigen::VectorXd V(num_elem);
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			for(int i=0; i<(int)Graph[*vp.first].getDOFvec().size() ; i++){
				MinMaxPhi=Graph[*vp.first].getDOFvec()[i].getDHParameter().getMinMaxPhi();
				if( MinMaxPhi.first!=0.0 || MinMaxPhi.second!=0.0){ 
					V[k]=MinMaxPhi.second;
					k++;
				}
				
			}
		}
		return V;
	};

	bool Skeleton::update(SkeletonJoint &joint, int DOF, double phi_t, double score){
		Graph[joint.getVertex()].updatePhi(DOF,phi_t,score);
		(*StartJoint).update(Graph);
		return true;
	};

	bool Skeleton::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone){	// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone

		Q2::SkeletonGraph G;

		int i=0;
		Q2::JointVertex v_source;
		Q2::JointVertex v_target;

		// merge skeleton S1 in new skeleton G
		Q2::vertex_map vm;
		boost::copy_graph(S1.getGraph(),G);
		for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
			v_target=i;
			v_source=i;
			// update vertex in joints in G
			G[*vp.first].setVertex(v_target);
			// map vertices from old Skeleton to new Skeleon
			vm=S1.getMap();
			vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
			S1.setMap(vm);
		}

		
		
		// merge skeleton S2 in new skeleton G
		i=0;

		clock_t begin = clock();
		boost::copy_graph(S2.getGraph(),G);
		double elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;

		for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
			if(i >= (int)boost::num_vertices(S1.getGraph())){
				v_target=i;
				int q=boost::num_vertices(S1.getGraph());
				v_source=i - boost::num_vertices(S1.getGraph());
				// update vertex in joints in G
				G[*vp.first].setVertex(v_target);
				// map vertices from old Skeleton to new Skeleon
				vm=S2.getMap();
				vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
				S2.setMap(vm);
			}
		}
		/////// MAP last elemnet not correct!!!! //////
		

		// add edge to graph
		this->Graph=G;
		this->addBone(bone,G[v1],G[v2 + boost::num_vertices(S1.getGraph())]);

		// set DH parameter
		SkeletonJoint SJ;
		JointVertex v=vm.begin()->second;
		SJ=this->Graph[v];
		std::vector <JointDOF> JDOF;
		JDOF=SJ.getDOFvec();
		JDOF[0].setDHParameter(DH);
		SJ.setDOFvec(JDOF);
		this->Graph[v]=SJ;

		// set Skeleton mapping
		S1.setMapping(true);
		S1.setMappingSkeleton(*this);


		// set Skeleton mapping
		S2.setMapping(true);
		S2.setMappingSkeleton(*this);
		return true;
	};

	bool Skeleton::mergeSkeletonbyJointVertex(Skeleton& S1, JointVertex v1, Skeleton& S2, JointVertex v2){	// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone

		Q2::SkeletonGraph G;

		int i=0;
		Q2::JointVertex v_source;
		Q2::JointVertex v_target;

		int S1_num=(int)boost::num_vertices(S1.getGraph());
		int S2_num=(int)boost::num_vertices(S2.getGraph());

		// merge skeleton S1 in new skeleton G
		Q2::vertex_map vm;
		boost::copy_graph(S1.getGraph(),G);
		for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
			v_target=i;
			v_source=i;
			// update vertex in joints in G
			G[*vp.first].setVertex(v_target);
			// map vertices from old Skeleton to new Skeleon
			vm=S1.getMap();
			vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
			S1.setMap(vm);
		}

		
		
		// merge skeleton S2 in new skeleton G
		

		clock_t begin = clock();
		SkeletonGraph SG=S2.getGraph();
		Q2::JointVertex v_first=0;
		remove_vertex(v_first, SG);
		boost::copy_graph(SG,G);
		double elapsed_secs = double(clock() - begin) / CLOCKS_PER_SEC;

		vm=S2.getMap();
		v_source=v2;
		v_target=v1;
		vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
		S2.setMap(vm);
		i=0;
		for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
			if(i >= (int)boost::num_vertices(S1.getGraph())){
				v_target=i;
				int q=boost::num_vertices(S1.getGraph());
				v_source=i - boost::num_vertices(S1.getGraph()) + 1;
				// update vertex in joints in G
				G[*vp.first].setVertex(v_target);
				// map vertices from old Skeleton to new Skeleon
				vm=S2.getMap();
				vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
				S2.setMap(vm);
			}
		}
		
		JointVertex ut=S1_num; JointVertex vt=S1_num;
		BoneEdge BE= boost::edge(ut,vt,G).first;
		SkeletonBone SB=G[BE];
		boost::remove_edge(ut, vt, G);
		ut=v1; 
		vt=S1_num;
		add_edge(ut, vt, G);
		G[boost::edge(ut,vt,G).first]=SB;

		// add edge to graph
		this->Graph=G;
		//this->addBone(bone,G[v1],G[v2 + boost::num_vertices(S1.getGraph())]);

		/*
		// set DH parameter
		SkeletonJoint SJ;
		JointVertex v=vm.begin()->second;
		SJ=this->Graph[v];
		std::vector <JointDOF> JDOF;
		JDOF=SJ.getDOFvec();
		//JDOF[0].setDHParameter(DH);
		SJ.setDOFvec(JDOF);
		this->Graph[v]=SJ;
		*/

		// set Skeleton mapping
		S1.setMapping(true);
		S1.setMappingSkeleton(*this);


		// set Skeleton mapping
		S2.setMapping(true);
		S2.setMappingSkeleton(*this);
		return true;
	};

	bool Skeleton::mergeSkeletonbyJointVertex(JointVertex v1, Skeleton& S2, JointVertex v2, DHParameter DH, SkeletonBone bone){	// source skeleton, source vertex, target skeleton, target vertex, DH parameter, Bone

		int num_joints = boost::num_vertices(this->Graph);
		Q2::SkeletonGraph G;
		boost::copy_graph(this->Graph,G);
		int i=0;
		Q2::JointVertex v_source;
		Q2::JointVertex v_target;
		Q2::vertex_map vm;

		if(num_joints==0){
			// merge skeleton S1 with Skeleton
			i=0;
			boost::copy_graph(S2.getGraph(),G);
			for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
				v_target=i;
				v_source = i - num_joints - 1;
				// update vertex in joints in G
				G[*vp.first].setVertex(v_target);
				// map vertices from old Skeleton to new Skeleon
				vm=S2.getMap();
				vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
				S2.setMap(vm);
			}
			S2.setMapping(true);

			// add edge to graph
			this->Graph=G;

		}else{
			// merge skeleton S1 with Skeleton
			i=0;
			boost::copy_graph(S2.getGraph(),G);
			for (std::pair<vertex_iter, vertex_iter> vp = vertices(G); vp.first != vp.second; ++vp.first, i++){
				if(i >= num_joints){
					v_target=i;
					v_source = i - num_joints - 1;
					// update vertex in joints in G
					G[*vp.first].setVertex(v_target);
					// map vertices from old Skeleton to new Skeleon
					vm=S2.getMap();
					vm.insert( std::pair<Q2::JointVertex, Q2::JointVertex>(v_source,v_target) );
					S2.setMap(vm);
				}
			}

			// set Skeleton mapping
			S2.setMapping(true);
			S2.getMappingSkeleton()->setGraph(G);

			// add edge to graph
			this->Graph=G;
			this->addBone(bone,G[v1],G[v2 + num_joints]);
		}

		return true;
	};

	bool Skeleton::init(shapeType JointShapeType, shapeType BoneShapeType){

		// iterate over vertices
		std::pair<vertex_iter, vertex_iter> vp;
		for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
			//init joint based on joint shape
			SkeletonJoint SJ;
			SJ=Graph[*vp.first];
			SJ.init(JointShapeType);
			Graph[*vp.first]=SJ;
		}

		// iterate over edges
		edge_iter ei, ei_end;
		for (boost::tie(ei, ei_end) = edges(Graph); ei != ei_end; ++ei){
			// init bone based on bone shape
			SkeletonBone SB;
			SB=Graph[*ei];
			SB.init(BoneShapeType);
			Graph[*ei]=SB;
		}


		return true;
	};

	bool Skeleton::draw(){

		if((int)num_vertices(Graph) > 0 && (int)num_edges(Graph) > 0){
			// iterate over vertices
			std::pair<vertex_iter, vertex_iter> vp;
			for (vp = vertices(Graph); vp.first != vp.second; ++vp.first){
				//draw joint based on jooint shape
				Graph[*vp.first].getShape().draw();

			}

			// iterate over edges
			edge_iter ei, ei_end;
			int i=0;

			for (boost::tie(ei, ei_end) = edges(Graph); ei != ei_end; ++ei){
				// draw bone based on bone shape	
				//Graph[*ei].getShape().draw();

			}
			return true;
		}else{
			return false;
		}
	};

	SkeletonGraph& Skeleton::getGraph(){
		return Graph;
	};



}

