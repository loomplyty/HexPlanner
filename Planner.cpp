
#include "Planner.h"

using namespace Dynamics;


void StepPlanner::initPlanner()
{
    stepCount=0;
    count=0;
    memcpy(swingID,LegPair1,sizeof(swingID));
    memcpy(stanceID,LegPair2,sizeof(stanceID));
    for (int i:swingID)
    {
        legState[i]=Swing;
    }
    for (int i:stanceID)
    {
        legState[i]=Stance;
    }
    robotState=ThreeStance;
}


void StepPlanner::initStep(int* _swingID, int* _stanceID)
{
    stepCount+=1;
    count=0;
    memcpy(swingID,_swingID,sizeof(swingID));
    memcpy(stanceID,_stanceID,sizeof(stanceID));

    for (int i:swingID)
    {
        legState[i]=Swing;
    }
    for (int i:stanceID)
    {
        legState[i]=Stance;
    }
    robotState=ThreeStance;
}

void StepPlanner::setInitConfig(const Matrix<double,3,6>& _initLegPee, const Vector3d & _initBodyPee, const Matrix3d& _initBodyR)
{
    initM.LegPee=_initLegPee;
    initM.BodyPee=_initBodyPee;
    initM.BodyR=_initBodyR;
}
void StepPlanner::setTargetConfig(const Matrix<double,3,6>& _targetLegPee, const Vector3d & _targetBodyPee, const Matrix3d& _targetBodyR)
{
    targetM.LegPee=_targetLegPee;
    targetM.BodyPee=_targetBodyPee;
    targetM.BodyR=_targetBodyR;
}

void StepPlanner::setInitBodyV(const Vector3d& _initBodyV)
{
    initM.BodyVee=_initBodyV;
}
void StepPlanner::setTargetBodyV(const Vector3d& _targetBodyV)
{
    targetM.BodyVee=_targetBodyV;
}
void StepPlanner::setStepHeight(const double H)
{
    stepHeight=H;
}
void StepPlanner::setStepPeriod(const double T)
{
    totalCount=T*COUNT_PER_SEC;
}

void StepPlanner::setFeetForces(const Matrix<double,3,6>& _feetForces)
{
    currentF.FootForce=_feetForces;
}
void StepPlanner::setOnForceSensor()
{
    isForceSensorApplied=true;
}
void StepPlanner::setOffForceSensor()
{
    isForceSensorApplied=false;
}
void StepPlanner::setPlanFrequency(const double _freq)
{
    countPerPeriod=int(1/_freq*COUNT_PER_SEC);
}
int StepPlanner::getCount()
{
    return count;
}


void StepPlanner::PlanUpdate()
{
    count+=countPerPeriod;
    if (count == 0)
        lastM=initM;
    else
        lastM=currentM;
}
void StepPlanner::PlanRefTrajGeneration()
{
    if(count<=totalCount)
    {
        int swingCount=2*totalCount*(1-dutyF);
        //legs
        if (count<=swingCount)
        {
            for(int i:swingID)
            {
                currentM.LegPee.col(i)=PlanTrajEllipsoid(initM.LegPee.col(i),targetM.LegPee.col(i),stepHeight,count,totalCount);
            }
        }
        else
            for(int i:swingID)
                currentM.LegPee.col(i)=lastM.LegPee.col(i);

        for(int i:stanceID)
        {
            currentM.LegPee.col(i)=initM.LegPee.col(i);
        }
        //body
        currentM.BodyPee=PlanTrajEllipsoid(initM.BodyPee,targetM.BodyPee,0,count,totalCount);
        //or planned as cubic spline
        currentM.BodyPee=PlanTrajCubic(initM.BodyPee, targetM.BodyPee, initM.BodyVee,targetM.BodyVee, count,totalCount);

        currentM.BodyR=PlanRbyQuatInterp(initM.BodyR,targetM.BodyR,count,totalCount);
    }
    //    else
    //        currentM=lastM;

}

void StepPlanner::PlanTouchDownJudgement()
{
    if(isForceSensorApplied==true)
    {
        // leg state judging
        for (int i:swingID)
        {
            if(currentF.FootForce(2,i)>currentF.TDThreshold)//Fz
                legState[i]=Stance;
            else if(currentF.FootForce(2,i)<currentF.TDActivate)//Fz
                legState[i]=Swing;
            else
                legState[i]=Trans;
        }

        // robot state judging
        bool isAllLswStancing{true};
        bool isAllLswSwinging{true};
        for (int i:swingID)
        {
            if (legState[i]!=Stance)
                isAllLswStancing=false;
            if(legState[i]!=Swing)
                isAllLswSwinging=false;
        }
        if (isAllLswStancing==true)
            robotState=SixStance;
        else if (isAllLswSwinging==true)
            robotState=ThreeStance;
        else
            robotState=TransStance;
    }
    else// not using force sensor, use time as judgement for touching down
    {
        if(count>=2*totalCount*(1-dutyF))
        {
            for (int i:swingID)
                legState[i]=Stance;
            robotState=SixStance;
        }
        else
        {
            for (int i:swingID)
                legState[i]=Swing;
            robotState=ThreeStance;
        }
    }
}


void StepPlanner::PlanTrajModification()
{
    //1. stance Traj does not change
    //2. swing Traj stops when touching down, swing Traj continues if not touching down

    for (int i:swingID)
    {
        switch(legState[i])
        {
        case Stance:
            currentM.LegPee.col(i)=lastM.LegPee.col(i);
            break;
        case Swing:
            if(count>=2*totalCount*(1-dutyF))
            {
                Vector3d extendV=Vector3d(0,-0.03,0);
                currentM.LegPee.col(i)=lastM.LegPee.col(i)+countPerPeriod/COUNT_PER_SEC*extendV;
            }
            break;
        case Trans:// add impedance or slower motion here
            Vector3d extendV=Vector3d(0,-0.01,0);
            currentM.LegPee.col(i)=lastM.LegPee.col(i)+countPerPeriod/COUNT_PER_SEC*extendV;
            break;
        }
    }

    //3. body Traj, should also consider the robot velocity, but for now we dont do that.
    if(robotState!=SixStance&&count>totalCount)
    {
        //slow down the robot motion and wait for touching down
    }



    //add also a sqp trajectory modifier to compute optimal joint space velocities.
    // this need input variables(1.last configuration. 2. current configuration 3. robot kinematics/leg jacobian)
}
bool StepPlanner::PlanStepFinishJudgement()
{
    if(robotState==SixStance&&count>=totalCount)
        return true;
    else
        return false;
}


Vector3d PlanTrajEllipsoid(const Vector3d& p0,const Vector3d& p1,const double stepH,const int count, const int totalCount)
{
    double s;
    s = PI*(1 - cos(double(count)/ totalCount*PI)) / 2;//[0,PI]
    Vector3d axisShort(0, stepH, 0);
    Vector3d plannedPos = 0.5*(p0 + p1) + 0.5*(p0 - p1)*cos(s) + axisShort*sin(s);
    return plannedPos;
}
Matrix3d PlanRbyQuatInterp(const Matrix3d& R0,const Matrix3d& R1,const int count, const int totalCount)
{
    Quaterniond q0(R0);

    Quaterniond q1(R1);
    double s;
    s = PI*(1 - cos(double(count)/ totalCount*PI)) / 2;//[0,PI]

    Quaterniond plannedQ;
    plannedQ.x()= 0.5*(q0.x() + q1.x()) + 0.5*(q0.x() - q1.x())*cos(s);
    plannedQ.y()= 0.5*(q0.y() + q1.y()) + 0.5*(q0.y() - q1.y())*cos(s);
    plannedQ.z()= 0.5*(q0.z() + q1.z()) + 0.5*(q0.z() - q1.z())*cos(s);
    plannedQ.w()= 0.5*(q0.w() + q1.w()) + 0.5*(q0.w() - q1.w())*cos(s);


    return Matrix3d(plannedQ.normalized());
}
Vector3d PlanTrajCubic(const Vector3d& p0, const Vector3d& p1, const Vector3d& v0, const Vector3d& v1,const int count, const int totalCount)
{
    //    // get configuration via cubic interpolation
    double T=totalCount/COUNT_PER_SEC;
    Matrix4d MTime, MTime_inv;
    MTime<<0,0,0,1,
            T*T*T,T*T,T,1,
            0,0,1,0,
            3*T*T,2*T,1,0;
    MTime_inv=MTime.inverse();
    Vector4d Ax,Ay,Az;
    Ax=MTime_inv*Vector4d(p0(0),p1(0),v0(0),v1(0));
    Ay=MTime_inv*Vector4d(p0(1),p1(1),v0(1),v1(1));
    Az=MTime_inv*Vector4d(p0(2),p1(2),v0(2),v1(2));

    Vector3d p;
    p(0)=MTime.row(1)*Ax;
    p(1)=MTime.row(1)*Ay;
    p(2)=MTime.row(1)*Az;
}

void PlanTrajP2Inf(const Vector3d& p0, const Vector3d& v0,const Vector3d& vdesire,const int count, const int accCount,Vector3d& p, Vector3d& v)
{
    double T=double(accCount)/COUNT_PER_SEC;
    double t=double(count)/COUNT_PER_SEC;

    Vector3d acc=(vdesire-v0)/T;
    if (count<=accCount)
    {
        v=v0+acc*t;
        p=v0*t+0.5*acc*t*t;
    }
    else
    {
        v=vdesire;
        p=v0*T+0.5*acc*T*T+vdesire*(t-T);
    }

}

