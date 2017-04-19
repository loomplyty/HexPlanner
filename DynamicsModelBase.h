#ifndef DYNAMICSMODELBASE_H
#define DYNAMICSMODELBASE_H
#include "DynamicsBase.h"

using namespace Eigen;
#define MAXJOINTSUPPORT 5

namespace Dynamics
{
	enum JointType
	{
		Undefined = 0,
		Rx = 1,
		Ry = 2,
		Rz = 3,
		Px = 4,
		Py = 5,
		Pz = 6
	};
	class Link
	{
	public:
		int id;
		double m;
		Vector3d c;
		Matrix3d Ic;
		Matrix3d Io;
        Matrix3d mcI(double _m, const Vector3d& _c, const Matrix3d& _Ic);
		Matrix3d mcI();
	};
	class Joint
	{
	public:
		int id;
		int jPredecessor;
		//int jSuccessor;
		JointType jType{ JointType::Undefined };
		int jSupport[MAXJOINTSUPPORT];
		int jNs;
	public:
		Matrix4d xTree;
		bool isActive;
        Matrix4d tm{Matrix4d::Identity()};
		Matrix3d rm{Matrix3d::Identity()};
		Vector3d p{Vector3d(0,0,0)};
		Vector3d v{ Vector3d(0,0,0) };
		Vector3d a{ Vector3d(0,0,0) };
		Vector3d omega{ Vector3d(0,0,0) };
		Vector3d alpha{ Vector3d(0,0,0) };
		double q{0};
		double qd{0};
		double qdd{0};
		 Matrix<double, 3, 3> jJp, jJo;
		 Matrix<double, 3, 9> jJpAll, jJoAll;
	public:
		Joint();
		Joint(int _id, JointType _jType, int _jPredecessor);
        void setXTree(const Matrix4d & _xTree);
        void setXTree(const Vector3d& _p, const Vector3d& _euler, const char *eurType);
		void setID(int _id);
		void setParent(int _jPredecessor);
 		void setSupport(int _jNs, int* _jSupport);
		void setType(JointType _jType);
		Matrix4d getJointTM();
		void display();
	};

}
#endif
