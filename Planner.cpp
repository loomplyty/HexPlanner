
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


void StepPlanner::initStep(int* _swingID, int* _stanceID)// do it when count == 0
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

void StepPlanner::parseVelocityCommand(const CommandPlanar & cmd)
{

    //1.init motion from data of the last step

    initM.Yaw=0;
    initM.BodyPee.setZero();
    double bodyPitch=robotFB.imuData(1);//213 euler angles
    double bodyRoll=robotFB.imuData(2);
    initM.BodyR=s_euler2rm(Vector3d(initM.Yaw,bodyPitch,bodyRoll),"213");
    for(int i=0;i<6;i++)
        initM.LegPee.col(i)=initM.BodyPee+initM.BodyR*robotFB.legPee2B.col(i);

    initM.BodyVee=R_t1_2_t0.inverse()*lastM.BodyVee;
    initM.COGV=initM.BodyVee;

    PlanGetSupportPlane(initM.LegPee,stanceID,supportPlaneParams);//planning w.r.t. the support plane, or it can just be (0,1,0,-0.85)
    PlaneGetPointProjection(initM.BodyPee,supportPlaneParams,initM.COG);// assuming the cog is the all from the body


    //2. calc for target motion
    Matrix<double, 3, 6> stdLegPee2C;

    stdLegPee2C << -0.3, -0.45, -0.3, 0.3, 0.45, 0.3,
                0,  0,  0,  0,  0,  0,
            -0.65, 0, 0.65, -0.65, 0, 0.65;

    //planning in the absolute world ground coordinate system

    targetM.COGV=Vector3d(cmd.Vx,0,cmd.Vy);
    targetM.BodyVee=targetM.COGV;
    targetM.Yaw=cmd.Yaw;
    Matrix3d R_t_2_b;
    Vector3d p_t_2_b;

    PlanGetT2B(initM.LegPee,stanceID,R_t_2_b,p_t_2_b);

    Vector3d D=targetM.COGV*double(totalCount)/COUNT_PER_SEC;// should be larger than 0.5*
    // or :Vector3d D=0.5*(initM.COGV+targetM.COGV)*double(totalCount)/COUNT_PER_SEC;
    targetM.COG=initM.BodyR*(R_t_2_b*D/2+p_t_2_b);
    targetM.BodyPee=targetM.COG+Vector3d(0,0.85,0);

    //leg
    Vector3d plannedSW2T[3];
    for(int i=0;i<3;i++)
    {
        plannedSW2T[i]=s_roty2rm(targetM.Yaw)*stdLegPee2C.col(swingID[i])+3*D/4;
        targetM.LegPee.col(swingID[i])=initM.BodyR*(R_t_2_b*plannedSW2T[i]+p_t_2_b);
    }

    targetM.BodyR=initM.BodyR*R_t_2_b*s_roty2rm(targetM.Yaw-initM.Yaw);
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
void StepPlanner::setInitLegP(const Matrix<double,3,6>& _initLegPee)
{
    initM.LegPee=_initLegPee;
}
void StepPlanner::setTargetLegP(const Matrix<double,3,6>& _targetLegPee)
{
    targetM.LegPee=_targetLegPee;
}
void StepPlanner::setInitBodyP(const Vector3d& _initBodyP)
{
    initM.BodyPee=_initBodyP;
}
void StepPlanner::setTargetBodyP(const Vector3d& _targetBodyP)
{
    targetM.BodyPee=_targetBodyP;
}

void StepPlanner::setInitBodyV(const Vector3d& _initBodyV)
{
    initM.COGV=_initBodyV;
}
void StepPlanner::setTargetBodyV(const Vector3d& _targetBodyV)
{
    targetM.COGV=_targetBodyV;
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
void StepPlanner::setInitYaw(const double _initYaw)
{
    initM.Yaw=_initYaw;
}
void StepPlanner::setTargetYaw(const double _targetYaw)
{
    targetM.Yaw=_targetYaw;
}
int StepPlanner::getCount()
{
    return count;
}


void StepPlanner::PlanUpdate(const Feedbacks& _fb)
{
    robotFB=_fb;
    if (count == 0)
    {
        initM.Yaw=0;
        initM.BodyPee.setZero();
        double bodyPitch=robotFB.imuData(1);//213 euler angles
        double bodyRoll=robotFB.imuData(2);
        initM.BodyR=s_euler2rm(Vector3d(0,bodyPitch,bodyRoll),"213");

        for(int i=0;i<6;i++)
            initM.LegPee.col(i)=initM.BodyPee+initM.BodyR*robotFB.legPee2B.col(i);

        initM.COGV=s_roty2rm(-targetM.Yaw)*targetM.COGV;
        currentM=initM;
    }
    lastM=currentM;
    //count+=countPerPeriod;
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

void StepPlanner::PlanPeriodDone()
{
    count+=countPerPeriod;
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

void PlanGetSupportPlane(const Matrix<double,3,6>& legPee,int* stanceID,Vector4d& planeABCD)
{
    Vector3d p[3];
    double l[3];
    for(int i=0;i<3;i++)
        p[i]=legPee.col(stanceID[i]);

    planeABCD.block(0,0,3,1)=(p[2]-p[1]).cross(p[2]-p[0]);
    planeABCD(3)=-planeABCD.block(0,0,3,1).dot(p[0]);
    planeABCD=planeABCD/(planeABCD(1));

    //Ax+By+Cz+D=0   B=1
}

void PlaneGetPointProjection(const Vector3d& p, const Vector4d& planeABCD,Vector3d& projP)
{
    projP(0)=p(0);
    projP(2)=p(2);
    projP(1)=-(p(0)*planeABCD(0)+p(2)*p(2)+p(3))/planeABCD(1);
}

void PlanGetT2B(const Matrix<double,3,6>& legPee,int* stanceID,Matrix3d & R_t_2_b,Vector3d& p_t_2_b )
{

    Vector3d p[3];
    double l[3];
    for(int i=0;i<3;i++)
        p[i]=legPee.col(stanceID[i]);

    l[0]=(p[2]-p[1]).norm();
    l[1]=(p[0]-p[2]).norm();
    l[2]=(p[1]-p[0]).norm();

    p_t_2_b=(p[0]*l[0]+p[1]*l[1]+p[2]*l[2])/(l[0]+l[1]+l[2]);

    Vector3d x,y,z;

    y=(p[2]-p[1]).cross(p[2]-p[0]);
    y.normalized();
    y=y*sgn(y(1));

    z=Vector3d(1,0,0).cross(y);
    z.normalized();
    x=z.cross(y);
    x.normalized();
    R_t_2_b.row(0)=x;
    R_t_2_b.row(1)=y;
    R_t_2_b.row(2)=z;

}

