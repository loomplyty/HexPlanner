#include"DynamicsModelBase.h"
#include <iostream>
//#include <cmath>
namespace Dynamics
{
    Matrix3d Link::mcI(double _m, const Vector3d& _c, const Matrix3d& _Ic)
	{
		m = _m;
		c = _c;
		Ic = _Ic;
		Io = Ic + m* s_skew(c)*(s_skew(c).transpose());
		return Io;
	}
	Matrix3d Link::mcI()
	{
		Io = Ic + m*(s_skew(c).transpose())*s_skew(c);
		return Io;
	}
	Joint::Joint()
	{
		id = 0;
		jType = JointType::Undefined;
		jPredecessor = 0;
 		xTree = Matrix4d::Identity(4, 4);
	}
	Joint::Joint(int _id, JointType _jType, int _jPredecessor)
	{
		id = _id;
		jType = _jType;
		jPredecessor = _jPredecessor;
 		xTree = Matrix4d::Identity(4, 4);
 	}
    void Joint::setXTree(const Matrix4d & _xTree)
	{
		xTree =_xTree;
	}
    void Joint::setXTree(const Vector3d& _p, const Vector3d& _euler, const char *eurType)
	{
		xTree = s_pe2pm(_p, _euler, eurType);
	}
	void Joint::setType(JointType _jType)
	{
		jType = _jType;
	}
	void Joint::setID(int _id)
	{
		id = _id;
	}
	void Joint::setParent(int _jPredecessor)
	{
		jPredecessor = _jPredecessor;
	}
 
	void Joint::setSupport(int _jNs, int* _jSupport)
	{
		jNs = _jNs;
		memcpy(jSupport, _jSupport, jNs * sizeof(int));
	}
	Matrix4d Joint::getJointTM()
	{
		Matrix4d m(Matrix4d::Identity());
		switch (jType)
		{
		case JointType::Rx:
			m = s_rotx2pm(q);
			break;
		case JointType::Ry:
			m = s_roty2pm(q);
			break;
		case JointType::Rz:
			m = s_rotz2pm(q);
			break;
		case JointType::Px:
			m = s_trlt2pm(Vector3d(q, 0, 0));
			break;
		case JointType::Py:
			m = s_trlt2pm(Vector3d(0, q, 0));
			break;
		case JointType::Pz:
			m = s_trlt2pm(Vector3d(0, 0, q));
			break;
		default:
			std::cout << "Unrecognized joint type!" << std::endl;
			break;
		}
		return m;

	}
	void Joint::display()
	{
		std::cout << "JointType: " << jType << std::endl;
		std::cout << "ID: " << id << std::endl << "Parent: " << jPredecessor << std::endl;
		std::cout << "Support: ";
		for (int i = 0; i < jNs; i++)
			std::cout << " " << jSupport[i];
		std::cout << std::endl;
		std::cout << "Transformation from last body: " << std::endl << xTree << std::endl << std::endl;
	}

}
