#include <iostream>
#include <Eigen/Eigen>
#include "Dynamics.h"
#include <time.h>
#include <fstream>
#include "solver.h"
#include "Planner.h"
using namespace std;
using namespace Eigen;
using namespace Dynamics;

int main(int argc, char* argv[])
{
    static Solver FootForceSolver;

    //Vars vars;
    //Params params;
    //Workspace work;
    //Settings settings;
    //int num_iters;
    //set_defaults();
    //setup_indexing();
    //load_default_data();
    ///* Solve problem instance for the record. */
    //settings.verbose = 1;
    //num_iters = solve();

    clock_t start, finish;// , t1, t2, t3, t4, t5, t6, t7, t8, t9;

    //*************************** full-body acc, vel, pos test, ok!********************************//
    //	 HexRobot robot;
    //	robot.HexInit();
    //	Matrix<double, 6, 3> p0, p1;
    //p0 <<
    //	-0.3, -0.85, -0.65,
    //	-0.45, -0.85, 0,
    //	-0.3, -0.85, 0.65,
    //	0.3, -0.85, -0.65,
    //	0.45, -0.85, 0,
    //	0.3, -0.85, 0.65;
    //	Matrix<double, 3, 6> legPos, legVel, legAcc;
    //	legPos = p0.transpose();
    //	legVel = Matrix<double, 3, 6>::Zero();
    //	legAcc = Matrix<double, 3, 6>::Zero();

    // 	robot.setPeeB(Vector3d(0, 0, 0), Vector3d(0, 0.1, 0), "213");
    //	robot.setVeeB(Vector3d(0, 0, 0), Vector3d(0.2, 0, 0));
    //	robot.setAeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0));

    //	robot.setPeeL(legPos, 'G');
    //	robot.setVeeL(legVel, 'G');
    //	robot.setAeeL(legAcc , 'G');
    //	//cout << legAcc << endl;

    //	Matrix<double, 3, 6> qin, qdin, qddin;
    //	robot.getPin(qin);
    //	robot.getVin(qdin);
    //	robot.getAin(qddin);

    //
    //			cout << "qin" << endl;
    //			cout << qin << endl;
    //			cout << "qdin" << endl;
    //			cout << qdin << endl;
    //			cout << "qddin" << endl;
    //			cout << qddin << endl;
    ///////////////////////////////******************************///////////////////////////////////////
    //	 HexRobot robot;
    //	robot.HexInit();
    //		Matrix<double, 6, 3> p0, p1;
    //	p0 <<
    //		-0.3, -0.85, -0.65,
    //		-0.45, -0.85, 0,
    //		-0.3, -0.85, 0.65,
    //		0.3, -0.85, -0.65,
    //		0.45, -0.85, 0,
    //		0.3, -0.85, 0.65;
    //	p1 <<
    //	-0.3, -0.85, -0.35,
    //	-0.45, -0.85, 0,
    //	-0.3, -0.85, 1,
    //	0.3, -0.85, -0.65,
    //	0.45, -0.85, 0.3,
    //	0.3, -0.85, 0.65;

    //Matrix<double, 3, 6> legPos,legVel,legAcc;

    //Vector3d b0, b1, bPos,bVel,bAcc;
    //b0 << 0, 0, 0;
    //b1 << 0, 0.1, 0.2;

    //double H = 0.1;

    //string filename = "E:\\HexCodes\\Dynamics\\data.txt";

    //static ofstream file;
    //file.open(filename);
    //cout << "file is open" << file.is_open() << endl;
    //legPos = p0.transpose();
    //legVel.setZero();
    //legAcc.setZero();
    //Matrix<double, 3, 6> qin,qdin,qddin;
    //start = clock();

    //Vector3d leg0P, leg2P, leg4P, leg0V, leg2V, leg4V, leg0A, leg2A, leg4A;
    //for (int i = 0; i < 1000; i++)
    //{

    //	plan_bodyStep(b0, b1, i, 1000,bPos,bVel,bAcc);
    //	plan_legStep(Vector3d(p0.row(0)), Vector3d(p1.row(0)), 0.1, i, 1000, leg0P, leg0V, leg0A);
    //	plan_legStep(Vector3d(p0.row(2)), Vector3d(p1.row(2)), 0.1, i, 1000, leg2P, leg2V, leg2A);
    //	plan_legStep(Vector3d(p0.row(4)), Vector3d(p1.row(4)), 0.1, i, 1000, leg4P, leg4V, leg4A);
    //	legPos.col(0) = leg0P;
    //	legPos.col(2) = leg2P;
    //	legPos.col(4) = leg4P;
    //	legVel.col(0) = leg0V;
    //	legVel.col(2) = leg2V;
    //	legVel.col(4) = leg4V;
    //	legAcc.col(0) = leg0A;
    //	legAcc.col(2) = leg2A;
    //	legAcc.col(4) = leg4A;
    //	//cout << legVel.col(0) << endl;
    // 		robot.setPeeB(bPos, Vector3d(0, 0.1+double(i)/1000*0.2, 0), "213");
    //	robot.setVeeB(bVel, Vector3d(0.2, 0, 0));
    //	robot.setAeeB(bAcc, Vector3d(0, 0, 0));

    //	robot.setPeeL(legPos, 'G');
    //	robot.setVeeL(legVel, 'G');
    //	robot.setAeeL(legAcc, 'G');

    //	robot.getPin(qin);
    //	robot.getVin(qdin);
    //	robot.getAin(qddin);

    //	/*		  cout << "qin" << endl;
    //			  cout << qin << endl;*/
    //	for (int j = 0; j < 18; j++)
    //		file << qin(j) << " ";

    //	for (int j = 0; j < 18; j++)
    //		file << qdin(j) << " ";

    //	for (int j = 0; j < 18; j++)
    //		file << qddin(j) << " ";
    //	for (int j = 0; j < 18; j++)
    //		file << legVel(j) << " ";
    //	for (int j = 0; j < 18; j++)
    //		file << legAcc(j) << " ";
    //	file << endl;

    //}
    //file.close();
    //finish = clock();


    //Matrix<double, 3, 6> qin, qdin, qddin;

    //for (int i = 0; i <N; i++)
    //{

    // robot.setPeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0), "313");
    // robot.setVeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0));
    // robot.setAeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0));
    // robot.setPeeL(Matrix<double, 3, 6>(pleg.transpose()),'G');
    // robot.setVeeL(Matrix<double, 3, 6>(pleg.transpose()),'G');
    // robot.setAeeL(Matrix<double, 3, 6>(pleg.transpose()),'G');

    // robot.getPin(qin);
    // robot.getVin(qdin);
    // robot.getAin(qddin);
    // robot.updateStatus();
    // robot.calcJointTorque();
    //}


    //********************* dynamic force test on one leg*******************************************//
    HexRobot robot;
    robot.HexInit();
    Vector3d g(0, -9.8, 0);// = robot.legs[0].leg2BaseTree.block(0, 0, 3, 3)*Vector3d(9.8, 0, 0);
    robot.setGravity(g);
    //Matrix4d tree = robot.legs[0].leg2BaseTree;

    start = clock();
    for (int i = 0; i < 1; i++)
    {
        robot.setPeeB(Vector3d(0, 0, 0), Vector3d(0, 0, 0), "213");
        robot.setVeeB(Vector3d(0, 1, 0), Vector3d(0, 0, 0));
        robot.setAeeB(Vector3d(0, 0, 1), Vector3d(0, 0, 0));

        //	Matrix<double, 6, 3> p0, p1;
        //p0 <<
        //	-0.3, -0.85, -0.65,
        //	-0.45, -0.85, 0,
        //	-0.3, -0.85, 0.65,
        //	0.3, -0.85, -0.65,
        //	0.45, -0.85, 0,
        //	0.3, -0.85, 0.65;
        Matrix<double, 3, 6> p0;

        p0 << -0.3, -0.45, -0.3, 0.3, 0.45, 0.3,
                -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
                -0.65, 0, 0.65, -0.65, 0, 0.65;

        robot.setPeeL(p0, 'G');
        robot.setVeeL(Matrix<double, 3, 6>(Matrix<double, 3, 6>::Zero(3, 6)), 'G');
        robot.setAeeL(Matrix<double, 3, 6>(Matrix<double, 3, 6>::Zero(3, 6)), 'G');

        Matrix<double, 3, 6>qddin;
        //	robot.getAin(qddin);
        //	robot.updateStatus();
        //	robot.calcJointTorque();
        //	robot.calcResultantWrench();

        FootForceSolver.set_defaults();
        FootForceSolver.setup_indexing();
        for(int s;s<1;s++)
        {

            FootForceSolver.load_default_data();
            int num_iters;
            num_iters = FootForceSolver.solve();
        }
        //cout << "t" << *(FootForceSolver.vars.t) << endl;
        //for (int j = 0; j < 6; j++)
        //{
        //	robot2.legs[j].setPee(Vector3d(0.9140, 0.5673, -0.1872), 'L');
        //	robot2.legs[j].setVee(Vector3d(-0.5026, -1.3181, -0.2055), 'L');
        // 	robot2.legs[j].setAee(Vector3d(-2.9657, 1.9022, -0.0877), 'L');

        //	Vector3d Qin, Qdin, Qddin;
        //	robot2.legs[j].getPin(Qin);
        //	robot2.legs[j].getVin(Qdin); //ok
        //    robot2.legs[j].getAin(Qddin);//ok
        //}

        //cout << "qin" << endl;
        //cout << Qin << endl;
        //cout << "qdin" << endl;
        //cout << Qdin << endl;
        //cout << "qddin" << endl;
        //cout << Qddin << endl;
        //cout << "legs[0].m" << robot2.legs[0].links[2].m;
        //cout << "rmB" << robot2.rmB << endl;
        //cout << "omegaB" << robot2.omegaB << endl;

        //robot2.updateStatus();

        //robot2.calcJointTorque();

    }
    finish = clock();

    cout << "Mf" << robot.Mf << endl;
    cout << "totalF" << robot.resultantF << endl;
    cout << "totalM" << robot.resultantM << endl;
    cout << "jointTorque" << robot.jTorque << endl;

    //************************ test for leg kinematics good************************************//
    //Leg L1;
    //L1.setLegID(1);
    //L1.LegInit(tree);

    //L1.setPee(Vector3d(1, 0, 0),'L');
    //L1.setVee(Vector3d(0, 0, 0.1),'L');
    //L1.setAee(Vector3d(0.1, 0, 0),'L');
    ////Vector3d Qin, Qdin, Qddin;
    //L1.getPin(Qin);
    //L1.getVin(Qdin);
    //L1.getAin(Qddin);


    //  cout << "qin" << endl;
    //  cout << Qin << endl;
    //  cout << "qdin" << endl;
    //  cout << Qdin << endl;
    //  cout << "qddin" << endl;
    //  cout << Qddin << endl;
    //*********************************************************************************//
    //  cout << "joint torques" << robot.jTorque << endl;


    cout <<"clocks per sec:"<<CLOCKS_PER_SEC<< "time spent" << double(finish - start)/CLOCKS_PER_SEC*1000<<"ms" << endl;
    //cout << "dida" << CLOCKS_PER_SEC << endl;

    Matrix<double, 3, 6> initp,targetp;

    initp << -0.3, -0.45, -0.3, 0.3, 0.45, 0.3,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.65, 0, 0.65, -0.65, 0, 0.65;
    targetp << -0.3, -0.45, -0.3, 0.3, 0.45, 0.3,
            -0.85, -0.85, -0.85, -0.85, -0.85, -0.85,
            -0.3, 0, 0.35, -0.65, -0.3, 0.65;
    StepPlanner planner;

    planner.initPlanner();
    planner.setInitConfig(initp,Vector3d(0,0,0),s_rotx2rm(0.1));
    planner.setTargetConfig(targetp,Vector3d(0,0,0.2),Matrix3d::Identity());
    planner.setStepPeriod(1.0);
    planner.setPlanFrequency(100);

    for (int i=0;i<3;i++)
    {
        int swingID[3]{0,2,4};
        int stanceID[3]{1,5,3};
        planner.initStep(swingID,stanceID);

        while(1)
        {
            planner.PlanUpdate();
            cout<<"step Num:"<<i<< " swing leg 1: "<<planner.swingID[0]<<endl;
            planner.PlanRefTrajGeneration();
            //cout<<planner.currentConfig.LegPee<<endl;
            // cout<<"rotatation "<<planner.currentConfig.BodyR<<endl;
            planner.PlanTouchDownJudgement();
            planner.PlanTrajModification();
            bool isFinished=planner.PlanStepFinishJudgement();
            cout<<"count"<<planner.getCount()<<endl;//bodyPee"<<planner.currentM.BodyPee<<endl;
             if(isFinished==true)
                break;
        }
    }



    double aaa;
    std::cin >> aaa;
}


