#ifndef XIAOTIANARM_CONTROLLER_H
#define XIAOTIANARM_CONTROLLER_H

#include <mutex>
#include <thread>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "Timer.h"
#include "savemat.h"
#include "SharedMemory.h"
#include "SharedMemoryInterface.h"
#include "qpOASES.hpp"
#include "qpOASES/QProblem.hpp"
#include "RobotArmModel.h"
#include "ControlData.h"
#include "XiaotianArmCommData.h"
#include "Trajectory.h"
#include "UserInterface.h"

using namespace qpOASES;

class ArmController
{
public:
    ArmController();
    ~ArmController();

    void init();
    void run();
    void updateState();
    // void updateEstimation();
    
    void runController();

    void computeKinematicsDynamics();

    void updateCommand();
    void updateBuffer();

    void getQDes();
    void movearm();
    void getTestData();
    void userMoveArm();
    Eigen::Vector3d rot2rpy(Eigen::Matrix3d R);
    Eigen::Matrix3d rpy2rot(Eigen::Vector3d euler);

    int model_nv, model_nq;

    int k = 0, iter = 0;
    Eigen::VectorXd qdes_arm;
    Eigen::VectorXd dqdes_arm;
    Eigen::VectorXd q_end;
    Eigen::VectorXd q_initial;
    Eigen::VectorXd qdes_arm_last;
    Eigen::Vector3d Pdes_ref;
    Eigen::Vector3d Prot_ref;
    Eigen::Matrix3d P_rotation_ref;
    double gripper_q_ref;

    Eigen::Vector3d EE_end_pos;
    Eigen::Vector3d EE_initial_pos;

    SharedMemoryObject<SharedMemoryInterface> _sharedMemory;
    RobotArmModel robotarm;
    ArmData armdata;
    ArmCmd armcmd;
    LastState laststate;
    ArmParameter armParam;
    JointsState *_motorState;
    JointsCmd *_motorCmd;
    Timer time;
    Trajectory trajectory;
    UserInterface *interface;
};

#endif