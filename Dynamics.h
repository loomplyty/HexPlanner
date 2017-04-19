#ifndef DYNAMICS_H
#define DYNAMICS_H
#include <Eigen/Eigen>
#include "DynamicsModelBase.h"
#include "DynamicsBase.h"


using namespace Eigen;
namespace Dynamics
{
 

	struct MotionPoint
	{
		int bodyId;
		Vector3d pR{Vector3d(0,0,0)}; // position relative to the attached body
		Matrix3d rm{Matrix3d::Identity()};
 		Vector3d p{Vector3d(0,0,0)};
		Vector3d v{ Vector3d(0,0,0) };
		Vector3d a{ Vector3d(0,0,0) };
		Vector3d omega{ Vector3d(0,0,0)};
		Vector3d alpha{ Vector3d(0,0,0)};
 	};
class Leg
	{
 	public: //leg topological parameters
		double D{ 0.270 };
		double d{ 0.074 };
		double H{ 0.234 };
		double h{ 0.059 };
		Matrix4d leg2BaseTree{Matrix4d::Zero()};
		int legId;
	public:
		MotionPoint legEE;
 		Vector3d fEE;
  		Link links[9];
		Joint joints[9]; 

 		Matrix<double, 9, 3> jvActive2All;
		Matrix<double, 9, 1> cActive2All;
		Matrix<double, 6, 9> jvConstraint;
		Matrix<double, 6, 1> cConstraint;
		Vector3d gravity;
	public:
		Leg();
		void setLegID(int _legId);
        void LegInit(const Matrix4d& _leg2BaseTree);
        void setGravity(const Vector3d& _g);
 
		// set the leg foot tip motion
        void setPee(const Vector3d& _p, const char frame);
        void setVee(const Vector3d& _pd, const char frame);
        void setAee(const Vector3d& _pdd, const char frame);
        void setPee(const Vector3d& _p);
        void setVee(const Vector3d& _pd);
        void setAee(const Vector3d& _pdd);
		// set the leg anypoint motion, used not so often
		//void setMrand(int _bodyID, Matrix4d& _xTree,Vector3d& _P);
		//void setMrand(int _bodyID, Matrix4d& _xTree, Vector3d& _P, Vector3d& _Pd);
		//void setMrand(int _bodyID, Matrix4d& _xTree, Vector3d& _P, Vector3d& _Pd, Vector3d& _Pdd);
        
		// get the leg foot tip motion 
		//void getPee(Vector3d &_q);
		//void getVee(Vector3d &_qd);
		//void getAee(Vector3d &_qdd);

		// get the leg anypoint motion
		//void getMrand(int _bodyID, Matrix4d& _xTree, Vector3d& _P);
		//void getMrand(int _bodyID, Matrix4d& _xTree, Vector3d& _P, Vector3d& _Pd);
		//void getMrand(int _bodyID, Matrix4d& _xTree, Vector3d& _P, Vector3d& _Pd, Vector3d& _Pdd);

		//set the joint motion input L1, L2 and L3
		//void setPin(Vector3d &_q);
		//void setVin(Vector3d &_qd);
		//void setAin(Vector3d &_qdd);
		// get the joint motion L1, L2, L3
		void getPin(Vector3d &_q);
		void getVin(Vector3d &_qd);
		void getAin(Vector3d &_qdd);

		//get the foot tip jacobian w.r.t. active joints L1,L2,L3
		void getJvEE(Matrix3d & _Jp, Matrix3d &_Jo);
		void getCEE(Vector3d & _Jp, Vector3d &_Jo);

		//get the anypoint jacobian w.r.t. active joints L1,L2,L3
		void getJvRand(int _bodyID, Vector3d& _pLocal,Matrix3d & _Jp, Matrix3d &_Jo);
		void getCRand(int _bodyID, Vector3d& _pLocal, Vector3d & _Cp, Vector3d &_Co);

		void getFin(Vector3d& _fIn);
        void setFee(const Vector3d& _fEE);

		// IK calc all nine joints from pee
		void calcIK();
		void calcIKd();
		void calcIKdd();
		//FK, calc pee and other six joints from L1,L2,L3, has recursive IK inside
		void calcFK();
 		void calcFKd();
		void calcFKdd();

		void calcVelConstraint();
		void calcAccConstraint();

        void calcJac(int _bodyID, const Vector3d& _p, Matrix<double,3,9> & _Jp,  Matrix<double, 3, 9> &_Jo);
        void calcC(int _bodyID, const Vector3d& _p,  Vector3d & _Cp,  Vector3d &_Co);
	};

class HexRobot
{
private: //parameters
	//Matrix<double, 3, 6> hipPee2B;
	//Matrix3d hipTreeRm[6];
	//Vector3d hipTreeP[6];
public: 
	Matrix3d rmB{Matrix3d::Identity()};
	Matrix4d pmB{ Matrix4d::Identity()};
	Vector3d pB{ Vector3d(0,0,0) };
	Vector3d vB{ Vector3d(0,0,0) };
	Vector3d aB{ Vector3d(0,0,0) };
	Vector3d omegaB{Vector3d(0,0,0)};
	Vector3d alphaB{ Vector3d(0,0,0) };
	Leg legs[6];
	Link body;
	Vector3d cB{ Vector3d(0,0,0) };
	double mB{ 0 };
	Matrix3d Io{ Matrix3d::Zero() }, Ic{ Matrix3d::Zero()};
	Vector3d gravity{Vector3d(0,-9.8,0)};
	
	Matrix<double, 3, 6> FeetForces;
	Vector3d resultantF, resultantM;
	Matrix<double, 6, 18> Mf;
private:
	Matrix<double, 3, 6> Pleg;
	Matrix<double, 3, 6> Vleg;
	Matrix<double, 3, 6> Aleg;

	Matrix<double, 3, 6> PlegA;
	Matrix<double, 3, 6> PlegR;

	Matrix<double, 3, 6> VlegR;
	Matrix<double, 3, 6> VlegA;

	Matrix<double, 3, 6> AlegA;
	Matrix<double, 3, 6> AlegR;
	Matrix<double, 3, 6> AlegCf;
	Matrix<double, 3, 6> AlegCl;

	Matrix<double, 3, 6> Pleg2B;
	Matrix<double, 3, 6> Vleg2B;
	Matrix<double, 3, 6> Aleg2B;

public:
	HexRobot();
	void HexInit();
    void setPeeL(const Matrix<double,3,6>& _legP,const char frame);
    void setVeeL(const Matrix<double, 3, 6>& _legPd, const char frame);
    void setAeeL(const Matrix<double, 3, 6>& _legPdd, const char frame);

    void setPeeB(const Vector3d& _pB, const Vector3d& _eulerB, const char *eurType);
    void setPeeB(const Vector3d& _pB, const Matrix3d& _rmB);

    void setVeeB(const Vector3d& _pdB,const Vector3d &_omegaB);
    void setAeeB(const Vector3d &_pddB, const Vector3d& _alphaB);

	void getPin(Matrix<double, 3, 6>& _q);
	void getVin(Matrix<double, 3, 6>& _qd);
	void getAin(Matrix<double, 3, 6>& _qdd);
	 
	void calcIK();
	void calcIKd();
	void calcIKdd();
  
	void setGravity(Vector3d& _g);
  
	void setFee(Matrix<double,3,6> & _FeetForces);
	void getFin(Matrix<double,3,6> & _Fin);
public: // for dynamics calculation
 	//Matrix<double, 3, 9> Acc[6];
	Matrix<double, 3, 9> Fi[6]{ Matrix<double,3,9>::Zero() };
	Matrix<double, 3, 9> Mi[6]{ Matrix<double,3,9>::Zero() };
	Matrix<double, 3, 9> Fg[6]{ Matrix<double,3,9>::Zero() };
	Matrix<double, 3, 9> Mg[6]{ Matrix<double,3,9>::Zero() };
	Matrix<double, 3, 6 > jTorque{Matrix<double,3,6>::Zero()};
	Matrix<double, 3, 6> fForce{ Matrix<double,3,6>::Zero() };
public:
 	void updateStatus();
	Vector3d getJointOmega(int _legId, int _jointId);
	Vector3d getJointAcc(int _legId, int _jointId);
	Vector3d getJointAlpha(int _legId, int _jointId);
	void setFeetForces(Matrix<double, 3, 6>& _fForce);
	void calcJointTorque();
	void calcResultantWrench();
  
	// NEWTON EQUATION: Fi+Fg+Fext=0
	// EULER EQUATION: Mi+Mg+Mext=0
	// Leg Local Dynamics Fi+Fg+Mf*Fext+torque=0
 
};


 
}
#endif
