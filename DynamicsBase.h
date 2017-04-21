#ifndef DYNAMICSBASE_H
#define DYNAMICSBASE_H
#include <Eigen/Eigen>
using namespace Eigen;
#define PI 3.1415926

 namespace Dynamics
 {
	 // mathematic calculation section
	 Matrix3d s_rotx2rm(double _theta);
	 Matrix3d s_roty2rm(double _theta);
	 Matrix3d s_rotz2rm(double _theta);
     Matrix3d s_euler2rm(const Vector3d& _euler, const char *eurType);

	 Matrix4d s_rotx2pm(double _theta);
	 Matrix4d s_roty2pm(double _theta);
	 Matrix4d s_rotz2pm(double _theta);
     Matrix4d s_trlt2pm(const Vector3d& _pos);
     Matrix4d s_euler2pm(const Vector3d& _euler, const char *eurType);
     Matrix4d s_pe2pm(const Vector3d& _p, const Vector3d& _euler, const char *eurType);
     Matrix3d s_skew(const Vector3d& _vec);
     int sgn(const double& d);
//     Matrix3d s_q2rm(const Vector4d& _quat);
//     Matrix4d s_q2pm(const Vector4d& _quat);
    // Quaterniond s_rm2q(const Matrix3d& _rm);
   //  Quaterniond s_pm2q(const Matrix4d& _pm);

	 // planning functions 

     void plan_legStep(const Vector3d& p0,const Vector3d& p1, const double stepH,const int count, const int totalCount, Vector3d & legPos);
     void plan_legStep(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d & legPos, Vector3d & legVel);
     void plan_legStep(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d & legPos, Vector3d & legVel, Vector3d & legAcc);

     void plan_bodyStep(const Vector3d& p0, const Vector3d& p1,const int count, const int totalCount, Vector3d& bodyPos);
     void plan_bodyStep(const Vector3d& p0,const  Vector3d& p1, const int count, const int totalCount, Vector3d& bodyPos, Vector3d& bodyVel);
     void plan_bodyStep(const Vector3d& p0, const Vector3d& p1, const int count, const int totalCount, Vector3d& bodyPos, Vector3d& bodyVel, Vector3d& bodyAcc);
	 


	/* Vector3d s_PGlobalFB(Vector3d& _pBase, Matrix3d &_rmBase, Vector3d& _pR);
	 Vector3d s_VGlobalFB(Vector3d& _pBase, Vector3d& _pdBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _pR, Vector3d& _pdR);
	 Vector3d s_AGlobalFB(Vector3d& _pBase, Vector3d& _pdBase, Vector3d& _pddBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _alphaBase, Vector3d& _pR, Vector3d& _pdR, Vector3d& _pddR);
	 Vector3d s_PLocalFB(Vector3d& _pBase, Matrix3d &_rmBase, Vector3d& _p);
	 Vector3d s_VLocalFB(Vector3d& _pBase, Vector3d& _pdBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _p, Vector3d& _pd);
	 Vector3d s_ALocalFB(Vector3d& _pBase, Vector3d& _pdBase, Vector3d& _pddBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _alphaBase, Vector3d& _p, Vector3d& _pd, Vector3d& _pdd);
*/
 }
#endif
