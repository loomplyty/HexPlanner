#include "Dynamics.h"
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#define PI 3.1415926
using namespace Eigen;
namespace Dynamics
{
Matrix3d s_rotx2rm(double _theta)
{
    return AngleAxisd(_theta, Vector3d::UnitX()).toRotationMatrix();
}
Matrix3d s_roty2rm(double _theta)
{
    return AngleAxisd(_theta, Vector3d::UnitY()).toRotationMatrix();
}
Matrix3d s_rotz2rm(double _theta)
{
    return AngleAxisd(_theta, Vector3d::UnitZ()).toRotationMatrix();
}

Matrix3d s_euler2rm(const Vector3d& _euler, const char *eurType)
{
    Vector3d axis[3];
    for (int i = 0; i < 3; i++)
    {
        if (eurType[i] == '1')
            axis[i] = Vector3d::UnitX();
        else if (eurType[i] == '2')
            axis[i] = Vector3d::UnitY();
        else if (eurType[i] == '3')
            axis[i] = Vector3d::UnitZ();
        else
        {
            std::cout << "Warning: inappropriate euler index, should be 1 or 2 or 3! " << std::endl;
            break;
        }
    }
    Matrix3d m;


    m = AngleAxisd(_euler(0), axis[0])*AngleAxisd(_euler(1), axis[1])*AngleAxisd(_euler(2), axis[2]);
    return m;
}

Matrix4d s_rotx2pm(double _theta)
{
    Matrix4d m;
    m << s_rotx2rm(_theta), Vector3d::Zero(),
            0, 0, 0, 1;
    return m;
}
Matrix4d s_roty2pm(double _theta)
{
    Matrix4d m;
    m << s_roty2rm(_theta), Vector3d::Zero(),
            0, 0, 0, 1;
    return m;
}
Matrix4d s_rotz2pm(double _theta)
{
    Matrix4d m;
    m << s_rotz2rm(_theta), Vector3d::Zero(),
            0, 0, 0, 1;
    return m;
}
Matrix4d s_trlt2pm(const Vector3d& _pos)
{
    Matrix4d m;
    m << Matrix3d::Identity(), _pos,
            0, 0, 0, 1;
    return m;
}
Matrix4d s_euler2pm(const Vector3d& _euler, const char *eurType)
{
    Matrix4d m;
    m << s_euler2rm(_euler, eurType), Vector3d::Zero(),
            0, 0, 0, 1;
    return m;
}
Matrix4d s_pe2pm(const Vector3d& _p, const Vector3d& _euler, const char *eurType)
{
    Matrix4d m;
    m << s_euler2rm(_euler, eurType), _euler,
            0, 0, 0, 1;
    return m;
}
Matrix3d s_skew(const Vector3d& _vec)
{
    Matrix3d m;
    m << 0, -_vec(2), _vec(1),
            _vec(2), 0, -_vec(0),
            -_vec(1), _vec(0), 0;
    return m;
}
Matrix3d s_q2rm(const Vector4d& _quat)
{
    Matrix3d rm_out;
    const double &q1 = _quat(0);
    const double &q2 = _quat(1);
    const double &q3 = _quat(2);
    const double &q4 = _quat(3);
    rm_out(0,0) = 1 - 2 * q2 * q2 - 2 * q3 * q3;
    rm_out(0,1) = 2 * q1 * q2 - 2 * q4 * q3;
    rm_out(0,2) = 2 * q1 * q3 + 2 * q4 * q2;

    rm_out(1,0) = 2 * q1 * q2 + 2 * q4 * q3;
    rm_out(1,1) = 1 - 2 * q1 * q1 - 2 * q3 * q3;
    rm_out(1,2) = 2 * q2 * q3 - 2 * q4 * q1;

    rm_out(2,0)= 2 * q1 * q3 - 2 * q4 * q2;
    rm_out(2,1) = 2 * q2 * q3 + 2 * q4 * q1;
    rm_out(2,2) = 1 - 2 * q1 * q1 - 2 * q2 * q2;
}

int sgn(const double& d)
{
    if (d==0)
        return 0;
    else if (d>0)
        return 1;
    else
        return -1;

}

//    Matrix4d s_q2pm(const Vector4d &_quat)
//    {
//        Matrix3d pm_out;
//        const double &q1 = _quat(0);
//        const double &q2 = _quat(1);
//        const double &q3 = _quat(2);
//        const double &q4 = _quat(3);
//        pm_out(0,0) = 1 - 2 * q2 * q2 - 2 * q3 * q3;
//        pm_out(0,1) = 2 * q1 * q2 - 2 * q4 * q3;
//        pm_out(0,2) = 2 * q1 * q3 + 2 * q4 * q2;
//        pm_out(0,3)=0;

//        pm_out(1,0) = 2 * q1 * q2 + 2 * q4 * q3;
//        pm_out(1,1) = 1 - 2 * q1 * q1 - 2 * q3 * q3;
//        pm_out(1,2) = 2 * q2 * q3 - 2 * q4 * q1;
//        pm_out(1,3)=0;

//        pm_out(2,0)= 2 * q1 * q3 - 2 * q4 * q2;
//        pm_out(2,1) = 2 * q2 * q3 + 2 * q4 * q1;
//        pm_out(2,2) = 1 - 2 * q1 * q1 - 2 * q2 * q2;
//        pm_out(2,3)=0;

//        pm_out(3,0) = 0;
//        pm_out(3,1) = 0;
//        pm_out(3,2) = 0;
//        pm_out(3,3) = 1;

//    }

//    Quaterniond s_rm2q(const Matrix3d& _rm)
//    {


//        return Quaterniond(_rm);


//    }
//    Quaterniond s_pm2q(const Matrix4d& _pm)
//    {


//    }

void plan_legStep(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d &legPos)
{
    double s;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]
    Vector3d axisShort(0, stepH, 0);
    legPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s) + axisShort*sin(s);
}

void plan_legStep(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d & legPos, Vector3d & legVel)
{
    double s,sd;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI], 0.5*pi*(1-cos(t/T*pi))
    sd = 0.5*PI*sin(double(count) / totalCount*PI)*(PI / (double(totalCount)/1000));
    Vector3d axisShort(0, stepH, 0);
    legPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s) + axisShort*sin(s);
    legVel = -0.5*(p0 - p1)*sin(s)*sd+axisShort*cos(s)*sd;
    //	std::cout << legVel << std::endl;
}

void plan_legStep(const Vector3d& p0, const Vector3d& p1, const double stepH, const int count, const int totalCount, Vector3d & legPos, Vector3d & legVel, Vector3d & legAcc)
{
    double s, sd,sdd;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI], 0.5*pi*(1-cos(t/T*pi))
    sd = 0.5*PI*sin(double(count) / totalCount*PI)*(PI / (double(totalCount) / 1000));
    sdd= 0.5*PI*cos(double(count) / totalCount*PI)*(PI / (double(totalCount) / 1000))*(PI / (double(totalCount) / 1000));


    Vector3d axisShort(0, stepH, 0);

    legPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s) + axisShort*sin(s);
    legVel = -0.5*(p0 - p1)*sin(s)*sd+axisShort*cos(s)*sd;
    legAcc = -0.5*(p0 - p1)*cos(s)*sd*sd-0.5*(p0 - p1)*sin(s)*sdd -axisShort*sin(s)*sd*sd+ axisShort*cos(s)*sdd;
    //std::cout << legVel << std::endl;
}

void plan_bodyStep(const Vector3d& p0, const Vector3d& p1, const int count, const int totalCount, Vector3d& bodyPos)
{
    double s;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]
    bodyPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s);
}

void plan_bodyStep(const Vector3d& p0, const Vector3d& p1, const int count, const int totalCount, Vector3d& bodyPos, Vector3d& bodyVel)
{
    double s,sd;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]
    sd = 0.5*PI*sin(double(count) / totalCount*PI)*(PI / (double(totalCount) / 1000));

    bodyPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s);
    bodyVel = -0.5*(p0 - p1)*sin(s)*sd;
}
void plan_bodyStep(const Vector3d& p0, const Vector3d& p1, const int count, const int totalCount, Vector3d& bodyPos, Vector3d& bodyVel, Vector3d& bodyAcc)
{
    double s, sd, sdd;
    s = PI*(1 - cos(double(count) / totalCount*PI)) / 2;//[0,PI]
    sd = 0.5*PI*sin(double(count) / totalCount*PI)*(PI / (double(totalCount) / 1000));
    sdd = 0.5*PI*cos(double(count) / totalCount*PI)*(PI / (double(totalCount) / 1000))*(PI / (double(totalCount) / 1000));

    bodyPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s);
    bodyVel = -0.5*(p0 - p1)*sin(s)*sd;
    bodyAcc = -0.5*(p0 - p1)*cos(s)*sd*sd - 0.5*(p0 - p1)*sin(s)*sdd;
}




//Vector3d s_PGlobalFB(Vector3d& _pBase, Matrix3d &_rmBase, Vector3d& _pR)
//{
//	return _pBase + _rmBase*_pR;
//}
//Vector3d s_VGlobalFB(Vector3d& _pBase, Vector3d& _pdBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _pR, Vector3d& _pdR)
//{
//	return _pdBase + _omegaBase.cross(_pR) + _rmBase*_pdR;
//}
//Vector3d s_AGlobalFB(Vector3d& _pBase, Vector3d& _pdBase, Vector3d& _pddBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _alphaBase, Vector3d& _pR, Vector3d& _pdR, Vector3d& _pddR)
//{
//	return _pddBase + _alphaBase.cross(_pR) + _omegaBase.cross(_omegaBase.cross(_pR)) + 2 * _omegaBase.cross(_pdR) + _rmBase*_pddR;
//}
//Vector3d s_PLocalFB(Vector3d& _pBase, Matrix3d &_rmBase, Vector3d& _p)
//{
//	return _rmBase.inverse()*(_p - _pBase);
//}
//Vector3d s_VLocalFB(Vector3d& _pBase, Vector3d& _pdBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _p, Vector3d& _pd)
//{
//	Vector3d _pR = _rmBase.inverse()*(_p - _pBase);
//	return _rmBase.inverse()*(_pd-_pdBase- _omegaBase.cross(_pR));
//
//}
//Vector3d s_ALocalFB(Vector3d& _pBase, Vector3d& _pdBase, Vector3d& _pddBase, Matrix3d &_rmBase, Vector3d& _omegaBase, Vector3d& _alphaBase, Vector3d& _p, Vector3d& _pd, Vector3d& _pdd)
//{
//	Vector3d _pR = _rmBase.inverse()*(_p - _pBase);
//	Vector3d _pdR =_rmBase.inverse()*(_pd - _pdBase - _omegaBase.cross(_pR));
//	return _rmBase.inverse()*(_pdd- _pddBase- _alphaBase.cross(_pR) - _omegaBase.cross(_omegaBase.cross(_pR)) - 2 * _omegaBase.cross(_pdR));
//}


}
