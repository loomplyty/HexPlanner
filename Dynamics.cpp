#include "Dynamics.h"
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
using namespace Eigen;
namespace Dynamics
{
	Leg::Leg()
	{
	}
	void Leg::setLegID(int _legId)
	{
		legId = _legId;
	}
    void Leg::LegInit(const Matrix4d& _leg2BaseTree)
	{
		leg2BaseTree = _leg2BaseTree;
		// init for joints 
		JointType _jointTypes[9]{ JointType::Ry ,JointType::Rz,JointType::Px ,JointType::Ry, JointType::Rz, JointType::Px ,JointType::Ry,JointType::Rz,JointType::Px };
		int _jointParents[9]{ -1,0,1,-1,3,4,-1,6,7 };

		for (int i = 0; i < 9; i++)
		{
			joints[i].setID(i);
			joints[i].setParent(_jointParents[i]);
			joints[i].setType(_jointTypes[i]);
		}

		Matrix4d node1 = leg2BaseTree*s_trlt2pm(Vector3d(0, 0, 0));
		joints[0].setXTree(node1);
		joints[1].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));
		joints[2].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));
		Matrix4d node2 = leg2BaseTree*s_trlt2pm(Vector3d(0, H, D / 2));
		joints[3].setXTree(node2);
		joints[4].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));
		joints[5].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));
		Matrix4d node3 = leg2BaseTree*s_trlt2pm(Vector3d(0, H, -D / 2));
		joints[6].setXTree(node3);
		joints[7].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));
		joints[8].setXTree(s_trlt2pm(Vector3d(0, 0, 0)));

		int s1[1]{ 0 };
		joints[0].setSupport(1, s1);

		int s2[2]{ 0,1 };
		joints[1].setSupport(2, s2);

		int s3[3]{ 0,1,2 };
		joints[2].setSupport(3, s3);

		int s4[1]{ 3 };
		joints[3].setSupport(1, s4);

		int s5[2]{ 3,4 };
		joints[4].setSupport(2, s5);

		int s6[3]{ 3,4,5 };
		joints[5].setSupport(3, s6);

		int s7[1]{ 6 };
		joints[6].setSupport(1, s7);

		int s8[2]{ 6,7 };
		joints[7].setSupport(2, s8);

		int s9[3]{ 6,7,8 };
		joints[8].setSupport(3, s9);

		// init for leg end effector 
		legEE.bodyId = 2;
		legEE.pR = Vector3d(0.097, 0, 0);
		fEE.setZero();

		// init for links
		links[0].m = 0;
		links[1].m = 11;
		links[2].m = 5;
		links[3].m = 0;
		links[4].m = 11;
		links[5].m = 2;
		links[6].m = 0;
		links[7].m = 11;
		links[8].m = 2;
		links[0].c = Vector3d(0, 0, 0);
		links[1].c = Vector3d(82.5 / 1000, 0, 0);
		links[2].c = Vector3d(-20.0 / 1000, 0, 0);
		links[3].c = Vector3d(0, 0, 0);
		links[4].c = Vector3d(51.5 / 1000, 0, 0);
		links[5].c = Vector3d(-51.3 / 1000, 0, 0);
		links[6].c = Vector3d(0, 0, 0);
		links[7].c = Vector3d(51.5 / 1000, 0, 0);
		links[8].c = Vector3d(-51.3 / 1000, 0, 0);

		links[0].Ic <<
			0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		links[1].Ic <<
			0.3, 0, 0,
			0, 0.2, 0,
			0, 0, 0.02;
		links[2].Ic <<
			0.3, 0, 0,
			0, 0.3, 0,
			0, 0, 0.003;

		links[3].Ic <<
			0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		links[4].Ic = links[1].Ic;
		links[5].Ic <<
			0.09, 0, 0,
			0, 0.09, 0,
			0, 0, 0.0006;
		links[6].Ic <<
			0, 0, 0,
			0, 0, 0,
			0, 0, 0;
		links[7].Ic = links[1].Ic;
		links[8].Ic = links[5].Ic;
		for (int i = 0; i < 9; i++)
			links[i].mcI();
	}
    void Leg::setGravity(const Vector3d& _g)
	{
		gravity = _g;
	}
    void Leg::setPee(const Vector3d& _P)
	{
		legEE.p = _P;
		calcIK();
		calcFK();
	}
    void Leg::setPee(const Vector3d& _P, const char frame)
	{
		if (frame == 'B')
			legEE.p = _P;
		else if (frame == 'L')
			legEE.p = (leg2BaseTree*Vector4d(_P(0), _P(1), _P(2), 1)).block(0, 0, 3, 1);
		calcIK();
		calcFK();
		//std::cout << "legEE.p" << legEE.p << std::endl;
	}
    void Leg::setVee(const Vector3d& _Pd)
	{
		legEE.v = _Pd;
		calcVelConstraint();
		calcIKd();
		calcFKd();
	}
    void Leg::setVee(const Vector3d& _Pd, const char frame)
	{
		if (frame == 'B')
			legEE.v = _Pd;
		else if (frame == 'L')
			legEE.v = leg2BaseTree.block(0, 0, 3, 3)*_Pd;

		calcVelConstraint();
		calcIKd();
		calcFKd();
	}
    void Leg::setAee(const Vector3d& _Pdd)
	{
		legEE.a = _Pdd;
		calcAccConstraint();
		calcIKdd();
		calcFKdd();
	}
    void Leg::setAee(const Vector3d& _Pdd, const char frame)
	{
		if (frame == 'B')
			legEE.a = _Pdd;
		else if (frame == 'L')
			legEE.a = leg2BaseTree.block(0, 0, 3, 3)*_Pdd;
		calcAccConstraint();
		calcIKdd();
		calcFKdd();
	}

	void Leg::getPin(Vector3d &_q)
	{
		_q(0) = joints[2].q;
		_q(1) = joints[5].q;
		_q(2) = joints[8].q;
	}
	void Leg::getVin(Vector3d &_qd)
	{
		_qd(0) = joints[2].qd;
		_qd(1) = joints[5].qd;
		_qd(2) = joints[8].qd;
	}
	void Leg::getAin(Vector3d &_qdd)
	{
		_qdd(0) = joints[2].qdd;
		_qdd(1) = joints[5].qdd;
		_qdd(2) = joints[8].qdd;
	}
	void Leg::getJvEE(Matrix3d & _Jp, Matrix3d &_Jo)
	{
		//calc for L1,L2,L3 w.r.t.all joints
		//calc for jacobian w.r.t. all joints  and transform it to L1,L2,L3
		Matrix<double, 3, 9>  _JpAll, _JoAll;
		calcJac(2, Vector3d(0.097, 0, 0), _JpAll, _JoAll);
		_Jp = _JpAll*jvActive2All;
		_Jo = _JoAll*jvActive2All;
	}
	void Leg::getCEE(Vector3d & _Cp, Vector3d &_Co)
	{
		calcC(2, Vector3d(0.097, 0, 0), _Cp, _Co);
	}
	void Leg::getJvRand(int _bodyID, Vector3d& _pLocal, Matrix3d & _Jp, Matrix3d &_Jo)
	{
		Matrix<double, 3, 9>  _JpAll;
		Matrix<double, 3, 9>  _JoAll;
		calcJac(_bodyID, _pLocal, _JpAll, _JoAll);
		_Jp = _JpAll*jvActive2All;
		_Jo = _JoAll*jvActive2All;
	}
	void Leg::getCRand(int _bodyID, Vector3d& _pLocal, Vector3d & _Cp, Vector3d &_Co)
	{
		calcC(_bodyID, _pLocal, _Cp, _Co);
	}
	void Leg::calcIK()
	{
		Vector3d p2L = leg2BaseTree.block(0, 0, 3, 3).inverse()*(legEE.p - leg2BaseTree.block(0, 3, 3, 1));
		double x = p2L(0);
		double y = p2L(1);
		double z = p2L(2);
		double x2A = legEE.pR(0);
		double y2A = legEE.pR(1);
		double z2A = legEE.pR(2);

		joints[2].q = sqrt(x*x + y*y + z*z - y2A*y2A - z2A*z2A) - x2A;
		joints[1].q = asin(y / sqrt((joints[2].q + x2A)*(joints[2].q + x2A) + y2A*y2A)) - asin(y2A / sqrt((joints[2].q + x2A)*(joints[2].q + x2A) + y2A*y2A));
		joints[0].q = atan((z2A*x - ((joints[2].q + x2A)*cos(joints[1].q) - y2A*cos(joints[1].q))*z) / (((joints[2].q + x2A)*cos(joints[1].q) - y2A*sin(joints[1].q))*x + z2A*z));

		Matrix4d TM_A_2_L(s_roty2pm(joints[0].q)*s_rotz2pm(joints[1].q)*s_trlt2pm(Vector3d(joints[2].q, 0, 0)));
		Vector4d U2S2_2_L = TM_A_2_L*Vector4d(0, h, d / 2, 1) - Vector4d(0, H, D / 2, 1);
		Vector4d U3S3_2_L = TM_A_2_L*Vector4d(0, h, -d / 2, 1) - Vector4d(0, H, -D / 2, 1);
		joints[5].q = U2S2_2_L.norm();
		joints[8].q = U3S3_2_L.norm();
		joints[3].q = atan(-U2S2_2_L(2) / U2S2_2_L(0));
		joints[4].q = asin(U2S2_2_L(1) / joints[5].q);
		joints[6].q = atan(-U3S3_2_L(2) / U3S3_2_L(0));
		joints[7].q = asin(U3S3_2_L(1) / joints[8].q);
	}

	void Leg::calcIKd()
	{
		Matrix<double, 3, 9> _Jp, _Jo;
		calcJac(legEE.bodyId, legEE.pR, _Jp, _Jo);
		Matrix3d Jq2p, Jactive2q;
		Matrix<double, 9, 1> qdAll;
		Jq2p = _Jp.block(0, 0, 3, 3);
		Jactive2q = jvActive2All.block(0, 0, 3, 3);
		qdAll = jvActive2All*Jactive2q.inverse()*Jq2p.inverse()*legEE.v;
		for (int i = 0; i < 9; i++)
			joints[i].qd = qdAll(i);
	}

	void Leg::calcIKdd()
	{
		Matrix<double, 3, 9> _Jp, _Jo;
		Vector3d _Cp, _Co;
		calcJac(legEE.bodyId, legEE.pR, _Jp, _Jo);
		calcC(legEE.bodyId, legEE.pR, _Cp, _Co);

		Matrix3d Jq2p, Jactive2q;
		Jq2p = _Jp.block(0, 0, 3, 3);
		Jactive2q = jvActive2All.block(0, 0, 3, 3);

		Vector3d Cactive2q(cActive2All.block(0, 0, 3, 1));
		Matrix<double, 9, 1> qddAll;

		qddAll = jvActive2All*Jactive2q.inverse()*(Jq2p.inverse()*(legEE.a - _Cp) - Cactive2q) + cActive2All;
		for (int i = 0; i < 9; i++)
			joints[i].qdd = qddAll(i);
	}

	void Leg::calcFK()
	{
		Matrix4d Xr, XJ;
		//Xr->setZero();
		// in calc FK, update rm, pm and p for each Link
		for (int i = 0; i < 9; i++)
		{
			XJ = joints[i].getJointTM();
			Xr = joints[i].xTree*XJ;
			if (joints[i].jPredecessor == -1)
				joints[i].tm = Xr;
			else
				joints[i].tm = joints[joints[i].jPredecessor].tm*Xr;
			joints[i].rm = joints[i].tm.block(0, 0, 3, 3);
			joints[i].p = joints[i].tm.block(0, 3, 3, 1);
		}
	}

	void Leg::calcFKd()
	{
		Matrix<double, 9, 1> qdAll;
		qdAll.setZero();
		for (int i = 0; i < 9; i++)
		{
			qdAll(i) = joints[i].qd;
			calcJac(i, Vector3d(0, 0, 0), joints[i].jJpAll, joints[i].jJoAll);
			joints[i].v = joints[i].jJpAll*qdAll;
			joints[i].omega = joints[i].jJoAll * qdAll;
			joints[i].jJp = joints[i].jJpAll*jvActive2All;
			joints[i].jJo = joints[i].jJoAll*jvActive2All;
		}
	}
	void Leg::calcFKdd()
	{
		Matrix<double, 9, 1> qddAll;
		qddAll.setZero();
		Vector3d _Cp, _Co;
		for (int i = 0; i < 9; i++)
		{
			qddAll(i) = joints[i].qdd;
			calcC(i, Vector3d(0, 0, 0), _Cp, _Co);
			joints[i].a = joints[i].jJpAll * qddAll + _Cp;
			joints[i].alpha = joints[i].jJoAll * qddAll + _Co;
		}
	}

    void Leg::calcJac(int _bodyID, const Vector3d& _p, Matrix<double, 3, 9> & _Jp, Matrix<double, 3, 9> &_Jo) //ok
	{
		//static int nCallforJac;
		//nCallforJac++;
		//std::cout << "nCallforJac" << nCallforJac << std::endl;
		//! important, initialize matrices
		_Jp.setZero();
		_Jo.setZero();
		Vector3d z;
		// std::cout << "bodyid" << _bodyID << std::endl;
		// std::cout << "support num" << Nsp <<" "<<joints[_bodyID].jSupport[0]<< joints[_bodyID].jSupport[1]<< joints[_bodyID].jSupport[2]<<std::endl;
		Vector3d pend;
		pend = joints[_bodyID].rm*_p + joints[_bodyID].p;
		for (int i = 0; i < joints[_bodyID].jNs; i++)
		{
			Joint _joint = joints[joints[_bodyID].jSupport[i]];
			// std::cout << "jtype" << _joint.jType << std::endl;
			switch (_joint.jType)
			{
			case JointType::Rx:
				z = _joint.rm.col(0);//*Vector3d(1, 0, 0)
				_Jp.col(_joint.id) = z.cross(pend - _joint.p);
				_Jo.col(_joint.id) = z;
				break;
			case JointType::Ry:
				z = _joint.rm.col(1);
				_Jp.col(_joint.id) = z.cross(pend - _joint.p);
				_Jo.col(_joint.id) = z;
				break;
			case JointType::Rz:
				z = _joint.rm.col(2);
				_Jp.col(_joint.id) = z.cross(pend - _joint.p);
				_Jo.col(_joint.id) = z;
				break;
			case JointType::Px:
				z = _joint.rm.col(0);
				_Jp.col(_joint.id) = z;
				break;
			case JointType::Py:
				z = _joint.rm.col(1);
				_Jp.col(_joint.id) = z;
				break;
			case JointType::Pz:
				z = _joint.rm.col(2);
				_Jp.col(_joint.id) = z;
				break;
			default:
				std::cout << "unrecognized joint type. " << std::endl;
				break;
			}
		}
	}

    void Leg::calcC(int _bodyID, const Vector3d& _p, Vector3d & _Cp, Vector3d &_Co)
	{
		//static int nCallforC;
		//nCallforC++;
		//std::cout << "nCallforC" << nCallforC << std::endl;
		_Cp.setZero();
		_Co.setZero();
		Vector3d z;
		Vector3d pend, vend;
		pend = joints[_bodyID].rm*_p + joints[_bodyID].p;
		vend = joints[_bodyID].v + joints[_bodyID].omega.cross(joints[_bodyID].rm*_p);
		for (int i = 0; i < joints[_bodyID].jNs; i++)
		{
			Joint _joint = joints[joints[_bodyID].jSupport[i]];
			switch (_joint.jType)
			{
			case JointType::Rx:
				z = _joint.rm.col(0);
				_Cp += _joint.qd*((_joint.omega.cross(z)).cross(pend - _joint.p) + z.cross(vend - _joint.v));
				_Co += _joint.qd*(_joint.omega.cross(z));
				break;
			case JointType::Ry:
				z = _joint.rm.col(1);
				_Cp += _joint.qd*((_joint.omega.cross(z)).cross(pend - _joint.p) + z.cross(vend - _joint.v));
				_Co += _joint.qd*(_joint.omega.cross(z));
				break;

			case JointType::Rz:
				z = _joint.rm.col(2);
				_Cp += _joint.qd*((_joint.omega.cross(z)).cross(pend - _joint.p) + z.cross(vend - _joint.v));
				_Co += _joint.qd*(_joint.omega.cross(z));
				break;

			case JointType::Px:
				z = _joint.rm.col(0);
				_Cp += _joint.qd*(_joint.omega.cross(z));
				break;

			case JointType::Py:
				z = _joint.rm.col(1);
				_Cp += _joint.qd*(_joint.omega.cross(z));
				break;
			case JointType::Pz:
				z = _joint.rm.col(2);
				_Cp += _joint.qd*(_joint.omega.cross(z));
				break;

			default:
				std::cout << "unrecognized joint type. " << std::endl;
				break;
			}
		}
	}

	void Leg::calcVelConstraint()
	{
		// update jvActive2All

		// for the velocity field
		Matrix<double, 3, 9> Jp11, Jp12, Jp21, Jp22, Jo11, Jo12, Jo21, Jo22;

		calcJac(2, Vector3d(0, h, d / 2), Jp11, Jo11);
		calcJac(5, Vector3d(0, 0, 0), Jp12, Jo12);
		calcJac(2, Vector3d(0, h, -d / 2), Jp21, Jo21);
		calcJac(8, Vector3d(0, 0, 0), Jp22, Jo22);
		jvConstraint <<
			Jp11 - Jp12,
			Jp21 - Jp22;
		Matrix<double, 6, 6> JvConstraintPJ;
		JvConstraintPJ <<
			jvConstraint.col(0), jvConstraint.col(1), jvConstraint.col(3), jvConstraint.col(4), jvConstraint.col(6), jvConstraint.col(7);
		Matrix<double, 6, 3> JvConstraintAJ;
		JvConstraintAJ << jvConstraint.col(2), jvConstraint.col(5), jvConstraint.col(8);
		Matrix<double, 6, 3> jvActive2Passive(-JvConstraintPJ.inverse()*JvConstraintAJ);

		jvActive2All <<
			jvActive2Passive.row(0),
			jvActive2Passive.row(1),
			Matrix<double, 1, 3>(1, 0, 0),
			jvActive2Passive.row(2),
			jvActive2Passive.row(3),
			Matrix<double, 1, 3>(0, 1, 0),
			jvActive2Passive.row(4),
			jvActive2Passive.row(5),
			Matrix<double, 1, 3>(0, 0, 1);
	}
	void Leg::calcAccConstraint()
	{
		// update cActive2All, for the acceleration field
		Matrix<double, 6, 6> JvConstraintPJ;
		JvConstraintPJ <<
			jvConstraint.col(0), jvConstraint.col(1), jvConstraint.col(3), jvConstraint.col(4), jvConstraint.col(6), jvConstraint.col(7);
		Matrix<double, 6, 3> JvConstraintAJ;
		JvConstraintAJ << jvConstraint.col(2), jvConstraint.col(5), jvConstraint.col(8);

		Vector3d Cp11, Cp12, Cp21, Cp22, Co11, Co12, Co21, Co22;

		calcC(2, Vector3d(0, h, d / 2), Cp11, Co11);
		calcC(5, Vector3d(0, 0, 0), Cp12, Co12);
		calcC(2, Vector3d(0, h, -d / 2), Cp21, Co21);
		calcC(8, Vector3d(0, 0, 0), Cp22, Co22);

		cConstraint <<
			Cp11 - Cp12,
			Cp21 - Cp22;

		Matrix<double, 6, 1> cActive2Passive;
		cActive2Passive = -JvConstraintPJ.inverse()*cConstraint;
		cActive2All << cActive2Passive(0), cActive2Passive(1), 0, cActive2Passive(2), cActive2Passive(3), 0, cActive2Passive(4), cActive2Passive(5), 0;
	}

    void Leg::setFee(const Vector3d& _fEE)
	{
		fEE = _fEE;
	}
	void Leg::getFin(Vector3d& _fIn)
	{
		calcFKdd();
		Vector3d Finertia[9], Minertia[9], G[9], MG[9];
		Matrix<double, 3, 9>  FContribution;
		Matrix3d Io[9];
		setGravity(Vector3d(9.8, 0, 0));

		for (int i = 0; i < 9; i++)
		{
			Io[i] = joints[i].rm*links[i].Io*joints[i].rm.transpose();
			Finertia[i] = -links[i].m*joints[i].a;
			Minertia[i] = -(Io[i] * joints[i].alpha + joints[i].omega.cross(Io[i] * joints[i].omega));
			G[i] = links[i].m*gravity;
			MG[i] = (joints[i].rm*links[i].c).cross(G[i]);
			// std::cout << "Finertia[i]" << Finertia[i] << std::endl;
		   // std::cout << "Minertia[i]" << Minertia[i] << std::endl;
			//std::cout << "G[i]" << G[i] << std::endl;
			// std::cout << "MG[i]" << MG[i] << std::endl;

			FContribution.block(0, i, 3, 1) = joints[i].jJp.transpose()*(Finertia[i] + G[i]) + joints[i].jJo.transpose()*(Minertia[i] + MG[i]);
		}
		Matrix3d _JpEE, _JoEE;
		getJvEE(_JpEE, _JoEE);
		_fIn = -(FContribution.rowwise().sum() + _JpEE.transpose()*fEE);
	}
	HexRobot::HexRobot()
	{

	}
	void HexRobot::HexInit()
	{
		Matrix<double, 6, 3> hipPee2B;
		hipPee2B <<
			-0.0679295, 0, -0.354,
			-0.1234141, 0, 0,
			-0.0679295, 0, 0.354,
			0.0679295, 0, -0.354,
			0.1234141, 0, 0,
			0.0679295, 0, 0.354;
		double thetaYHip[6]{ PI*5.0 / 6.0,PI,PI*7.0 / 6.0,PI / 6.0,0,-PI / 6.0 };
		double thetaZHip[6]{ -PI*7.0 / 18.0,-PI*7.0 / 18.0,-PI*7.0 / 18.0 ,-PI*7.0 / 18.0 ,-PI*7.0 / 18.0,-PI*7.0 / 18.0 };

		Matrix4d legNodeTree;
		for (int i = 0; i < 6; i++)
		{
			legNodeTree = s_trlt2pm(Vector3d(hipPee2B.row(i)))*s_roty2pm(thetaYHip[i])*s_rotz2pm(thetaZHip[i]);
			legs[i].setLegID(i);
			legs[i].LegInit(legNodeTree);
		}
		mB = 50;
		cB = Vector3d(0, 0, 0);
		Ic = Matrix3d::Identity();
		Io = Ic + mB*(s_skew(cB).transpose())*s_skew(cB);
	}

    void HexRobot::setPeeL(const Matrix<double, 3, 6>& _legP, const char frame)
	{
		if (frame == 'G')
		{
			Pleg = _legP;
			calcIK();
			for (int i = 0; i < 6; i++)
				legs[i].setPee(Vector3d(Pleg2B.col(i)));
		}
		else if (frame == 'B')
		{
			Pleg2B = _legP;
			for (int i = 0; i < 6; i++)
				legs[i].setPee(Vector3d(Pleg2B.col(i)));
		}

	}
    void HexRobot::setVeeL(const Matrix<double, 3, 6>& _legPd, const char frame)
	{
		if (frame == 'G')
		{
			Vleg = _legPd;
			calcIKd();
			for (int i = 0; i < 6; i++)
				legs[i].setVee(Vector3d(Vleg2B.col(i)));
		}
		else if (frame == 'B')
		{
			Vleg2B = _legPd;
			for (int i = 0; i < 6; i++)
				legs[i].setVee(Vector3d(Vleg2B.col(i)));
		}

	}
    void HexRobot::setAeeL(const Matrix<double, 3, 6>& _legPdd, const char frame)
	{
		if (frame == 'G')
		{
			Aleg = _legPdd;
			calcIKdd();
			for (int i = 0; i < 6; i++)
				legs[i].setAee(Vector3d(Aleg2B.col(i)));
		}
		else if (frame == 'B')
		{
			Aleg2B = _legPdd;
			for (int i = 0; i < 6; i++)
				legs[i].setAee(Vector3d(Aleg2B.col(i)));
		}
	}
    void HexRobot::setPeeB(const Vector3d& _pB, const Vector3d& _eulerB, const char *eurType)
	{
		pB = _pB;
		rmB = s_euler2rm(_eulerB, eurType);
	}
    void HexRobot::setPeeB(const Vector3d& _pB, const Matrix3d& _rmB)
	{
		pB = _pB;
		rmB = _rmB;
	}
    void HexRobot::setVeeB(const Vector3d& _pdB, const Vector3d &_omegaB)
	{
		vB = _pdB;
		omegaB = _omegaB;
	}

    void HexRobot::setAeeB(const Vector3d &_pddB, const Vector3d& _alphaB)
	{
		aB = _pddB;
		alphaB = _alphaB;
	}
	void HexRobot::getPin(Matrix<double, 3, 6>& _q)
	{
		Vector3d _qIn;
		for (int i = 0; i < 6; i++)
		{
			legs[i].getPin(_qIn);
			_q.col(i) = _qIn;
		}
	}
	void HexRobot::getVin(Matrix<double, 3, 6>& _qd)
	{
		Vector3d _qdIn;
		for (int i = 0; i < 6; i++)
		{
			legs[i].getVin(_qdIn);
			_qd.col(i) = _qdIn;
		}
	}
	void HexRobot::getAin(Matrix<double, 3, 6>& _qdd)
	{
		Vector3d  _qddIn;
		for (int i = 0; i < 6; i++)
		{
			legs[i].getAin(_qddIn);
			_qdd.col(i) = _qddIn;
		}
	}
	void HexRobot::calcIK()
	{
		for (int i = 0; i < 6; i++)
		{
			PlegA.col(i) = pB;
			PlegR.col(i) = Pleg.col(i) - pB;
			Pleg2B.col(i) = rmB.inverse()*PlegR.col(i);
		}
	}
	void HexRobot::calcIKd()
	{
		for (int i = 0; i < 6; i++)
		{
			VlegA.col(i) = vB + omegaB.cross(PlegR.col(i));
			VlegR.col(i) = Vleg.col(i) - VlegA.col(i);
			Vleg2B.col(i) = rmB.inverse()*VlegR.col(i);
		}
	}
	void HexRobot::calcIKdd()
	{
		for (int i = 0; i < 6; i++)
		{
			AlegA.col(i) = aB + alphaB.cross(PlegR.col(i));
			AlegCl.col(i) = 2 * omegaB.cross(VlegR.col(i));
			AlegCf.col(i) = omegaB.cross(omegaB.cross(PlegR.col(i)));
			//AlegCf.col(i) = (omegaB.cross(omegaB)).cross(PlegR.col(i));// got a problem here; this equals to zero
			AlegR.col(i) = Aleg.col(i) - AlegA.col(i) - AlegCl.col(i) - AlegCf.col(i);
			Aleg2B.col(i) = rmB.inverse()*AlegR.col(i);
		}
	}

	void HexRobot::setGravity(Vector3d& _g)
	{
		gravity = _g;
	}
	void HexRobot::setFee(Matrix<double, 3, 6> & _FeetForces)
	{
		FeetForces = _FeetForces;
	}
	void HexRobot::getFin(Matrix<double, 3, 6> & _Fin)
	{
		_Fin = jTorque;
	}

	void HexRobot::updateStatus()
	{
		for (int i = 0; i < 6; i++)
		{
			for (int j : {1, 2, 4, 5, 7, 8})
			{
				Fi[i].col(j) = -legs[i].links[j].m*getJointAcc(i, j);
				Matrix3d I = (rmB*legs[i].joints[j].rm) *legs[i].links[j].Io*(rmB* legs[i].joints[j].rm).transpose();
				Mi[i].col(j) = -(I*getJointAlpha(i, j) + getJointOmega(i, j).cross(I*getJointOmega(i, j)));
				Fg[i].col(j) = legs[i].links[j].m*gravity;
				Mg[i].col(j) = (rmB*legs[i].joints[j].rm*legs[i].links[j].c).cross(Fg[i].col(j));
			}
		}
	}

	void HexRobot::calcJointTorque()
	{
		jTorque.setZero();
		for (int i = 0; i < 6; i++)
		{
			for (int j : { 1, 2, 4, 5, 7, 8 })
			{
				jTorque.col(i) += (rmB*legs[i].joints[j].jJp).transpose()*(Fi[i].col(j) + Fg[i].col(j));
				jTorque.col(i) += (rmB*legs[i].joints[j].jJo).transpose()*(Mi[i].col(j) + Mg[i].col(j));
			}
		}
	}
	void HexRobot::setFeetForces(Matrix<double, 3, 6>& _fForce)
	{
		fForce = _fForce;
	}

	void HexRobot::calcResultantWrench()
	{
		resultantF.setZero();
		resultantM.setZero();
		Mf.setZero();

		for (int i = 0; i < 6; i++)
		{
			for (int j : { 1, 2, 4, 5, 7, 8 })
			{
				resultantF += Fi[i].col(j) + Fg[i].col(j);
				resultantM += Mi[i].col(j) + Mg[i].col(j) + (rmB*legs[i].joints[j].p).cross(Fi[i].col(j) + Fg[i].col(j));
			}

			Mf.block(0, i * 3, 3, 3) = Matrix3d::Identity();
			Mf.block(3, i * 3, 3, 3) = s_skew(Vector3d(rmB*legs[i].legEE.p));

		}

		resultantF += mB*(gravity - aB);
		Matrix3d bodyI = rmB*Io*rmB.transpose();
		resultantM += -(bodyI*alphaB + omegaB.cross(bodyI*omegaB)) + (rmB*cB).cross(mB*gravity);

		resultantF = -resultantF;
		resultantM = -resultantM;
	}

	Vector3d HexRobot::getJointAlpha(int _legId, int _jointId)
	{
		Vector3d alphaA, alphaR;
		alphaA = alphaB + omegaB.cross(rmB*legs[_legId].joints[_jointId].omega);
		alphaR = rmB*legs[_legId].joints[_jointId].alpha;
		return alphaA + alphaR;
	}

	Vector3d HexRobot::getJointOmega(int _legId, int _jointId)
	{
		return omegaB + rmB*legs[_legId].joints[_jointId].omega;
	}
	Vector3d HexRobot::getJointAcc(int _legId, int _jointId)
	{
		Vector3d Acc, AccA, AccCf, AccCl, AccR;
		AccA = aB + alphaB.cross(rmB*legs[_legId].joints[_jointId].p);
		AccCl = 2 * omegaB.cross(rmB*legs[_legId].joints[_jointId].v);
		AccCf = omegaB.cross(omegaB.cross(rmB*legs[_legId].joints[_jointId].p));
		AccR = rmB*legs[_legId].joints[_jointId].a;
		Acc = AccA + AccCl + AccCf + AccR;

		return Acc;
	}
 
}

