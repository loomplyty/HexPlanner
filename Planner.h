#ifndef PLANNER_H
#define PLANNER_H

#include <Eigen/Eigen>
#include "Dynamics.h"
//#include <cmath>
#define COUNT_PER_SEC 1000
#define PI 3.1415926

using namespace Eigen;


const int LegPair1[3]{0,2,4};
const int LegPair2[3]{1,5,3};

struct RobotMotionStatus
{
    // w.r.t. the world coordinate system
    Matrix<double,3,6> LegPee{Matrix<double,3,6>::Zero()};
    Vector3d BodyPee{Vector3d::Zero()};
    Vector3d BodyVee{Vector3d::Zero()};
    Matrix3d BodyR{Matrix3d::Identity()};

    // w.r.t. the terrain coordinate system
    double Yaw{0};
    Vector3d COGV{Vector3d::Zero()};
    Vector3d COG{Vector3d::Zero()};
};

struct RobotForceStatus
{
    Matrix<double,3,6> FootForce{Matrix<double,3,6>::Zero()};
    double TDThreshold{300};
    double TDActivate{100};
};

struct Feedbacks
{
    Matrix<double,3,6> legPee2B{Matrix<double,3,6>::Zero()};
    Matrix<double,3,6> forceData{Matrix<double,3,6>::Zero()};
    Vector3d imuData{Vector3d::Zero()};
};

struct CommandPlanar
{
    double Vx;
    double Vy;
    double Yaw;
};


enum LegState
{
    Stance=0,
    Swing =1,
    Trans=2,
};
enum RobotState
{
    ThreeStance=3,
    TransStance=4,
    SixStance=6,
};

class StepPlanner  // update for each step
{
public:
    RobotMotionStatus initM;
    RobotMotionStatus targetM;
    RobotMotionStatus currentM;
    RobotMotionStatus lastM;

    RobotForceStatus currentF;
    Feedbacks robotFB;
    int swingID[3]{0,2,4};
    int stanceID[3]{1,5,3};

    double dutyF{0.6};
    LegState legState[6];
    RobotState robotState;
    int countPerPeriod{1};
    double stepHeight{0.05};
    Matrix3d R_t1_2_t0{Matrix3d::Identity()};
    Vector3d p_t1_2_t0{Vector3d(0,0,0)};
private:
    int stepCount{0};
    int count{0};
    int totalCount;
    bool isForceSensorApplied{false};
    Vector4d supportPlaneParams{Vector4d(0,1,0,0.85)};// default support plane and height in the World CS
public:


      // one-time functions
    void initPlanner();
    void initStep(int* _swingID, int* _stanceID);
    //setters
    void setInitConfig(const Matrix<double,3,6>& _initLegPee, const Vector3d & _initBodyPee, const Matrix3d& _initBodyR);
    void setTargetConfig(const Matrix<double,3,6>& _targetLegPee, const Vector3d & _targetBodyPee, const Matrix3d& _targetBodyR);
    void setInitLegP(const Matrix<double,3,6>& _initLegPee);
    void setTargetLegP(const Matrix<double,3,6>& _targetLegPee);
    void setInitBodyP(const Vector3d& _initBodyP);
    void setTargetBodyP(const Vector3d& _targetBodyP);
    void setInitBodyV(const Vector3d& _initBodyV);
    void setTargetBodyV(const Vector3d& _targetBodyV);
    void setInitYaw(const double _initYaw);
    void setTargetYaw(const double _targetYaw);
    void setStepHeight(const double H);
    void setStepPeriod(const double T);
    void setFeetForces(const Matrix<double,3,6>& _feetForces);
    void setOnForceSensor();
    void setOffForceSensor();
    void setPlanFrequency(const double _freq);

    void parseVelocityCommand(const CommandPlanar & cmd);

    //getters
    int getCount();
    // functions for each control loop

    // 0. count+=1
    void PlanUpdate(const Feedbacks& _fb);

    // 1. Generate Reference Trajectory
    //parse command..
    void PlanRefTrajGeneration();// a. field interactive traj b. steping over
    // 2. Detecting touch down
    void PlanTouchDownJudgement();// we need a function handler later to include various traj generation
    // 3. modify trajectory
    void PlanTrajModification();
    bool PlanStepFinishJudgement();
    void PlanPeriodDone();

};


Vector3d PlanTrajEllipsoid(const Vector3d& p0,const Vector3d& p1,const double stepH,const int count, const int totalCount);
Matrix3d PlanRbyQuatInterp(const Matrix3d& R0,const Matrix3d& R1,const int count, const int totalCount);
Vector3d PlanTrajCubic(const Vector3d& p0, const Vector3d& p1, const Vector3d& v0, const Vector3d& v1,const int count, const int totalCount);
void PlanTrajP2Inf(const Vector3d& p0, const Vector3d& v0,const Vector3d& vdesire,const int count, const int accCount,Vector3d& p, Vector3d& v);

void PlanGetT2B(const Matrix<double,3,6>& legPee,int* stanceID,Matrix3d & R_t_2_b,Vector3d& p_t_2_b );
void PlaneGetPointProjection(const Vector3d& p, const Vector4d& planeABCD,Vector3d& projP);
void PlanGetSupportPlane(const Matrix<double,3,6>& legPee,int* stanceID,Vector4d& planeABCD);





#endif
