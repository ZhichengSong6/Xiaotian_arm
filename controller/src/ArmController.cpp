# include "ArmController.h"

// #define TRAJECTORY
#define USERINPUT

ArmController::ArmController(){
    std::cout << "Enter the contsructor function" << std::endl;
    armParam.getParamsFromYAML("../../config/parameters.yaml");
    
    _sharedMemory.attach(ROBOTARM_SHARE_MEMORY_NAME);
    _sharedMemory().init();

    _motorState = &_sharedMemory().measuredState.jointsState;
    _motorCmd = &_sharedMemory().jointsCmd;

    interface = new UserInterface(&armParam);

    this->model_nq = robotarm.GetNq();
    this->model_nv = robotarm.GetNv();
}

void ArmController::init(){
    armParam.torquelimit << armParam.jointlimit[0], armParam.jointlimit[1], armParam.jointlimit[2], armParam.jointlimit[3], armParam.jointlimit[4], armParam.jointlimit[5], armParam.jointlimit[6];
    armdata.J.resize(6, this->model_nv);
    armdata.dJ.resize(6, this->model_nv);

    // armdata.J = robotarm.getJointJacobian();
    // std::cout << "succeefully get J! " << std::endl;
    // armdata.dJ = robotarm.getJacobDerivative();
    // std::cout << "succeefully get dJ! " << std::endl;

    Pdes_ref << armParam.des_translation[0], armParam.des_translation[1], armParam.des_translation[2];
    Prot_ref << armParam.des_rotation[0], armParam.des_rotation[1], armParam.des_rotation[2];
    P_rotation_ref = rpy2rot(Prot_ref);
    gripper_q_ref = armParam.gripper_des_q;

    this->qdes_arm.resize(6, 1);
    this->dqdes_arm.resize(6, 1);
    this->qdes_arm_last.resize(6, 1);
    this->q_end.resize(6, 1);
    this->q_initial.resize(6, 1);
    trajectory._jointMaxq.resize(6, 1);
    trajectory._jointMinq.resize(6, 1);
    trajectory._jointSpeedLimit.resize(6, 1);

    for(int i = 0; i < 6; ++i){
        trajectory._jointMaxq[i] = 1.54;
        trajectory._jointMinq[i] = -1.54;
        trajectory._jointSpeedLimit[i] = 1.57;
    }

    std::cout << "model_nv: " << robotarm.GetNv() << std::endl; 
    std::cout << "model_nq: " << robotarm.GetNq() << std::endl; 
    std::cout << "!!! initialize finished !!!" << std::endl;
}

void ArmController::run(){
    std::cout << "k: " << k << std::endl;

    interface->update();

    updateState();
    
    if(k == 0){
        init();
#ifdef TRAJECTORY
        std::cout << "generating trajectory!" << std::endl;
        getQDes();
        for(int i = 0; i < 6; i ++)
            q_initial[i] = armdata.arm_dq[i];
        trajectory.generateTraj(q_initial, q_end, 1);
        trajectory._startTime = time.getMs();
        std::cout << "Trajectory generate done!" << std::endl;
#endif

    }
    trajectory._currentTime = time.getMs();
    trajectory._T = (trajectory._currentTime - trajectory._startTime ) * pow(10, -3);
    runController();
    updateCommand();
    updateBuffer();

    // std::cout << "Torque    : " << armcmd.full_tau.transpose() << std::endl;

    k++;
}

void ArmController::runController(){

    computeKinematicsDynamics();

#ifdef TRAJECTORY
    movearm();
#endif

#ifdef USERINPUT
    if(k == 0){
        P_rotation_ref = robotarm.data.oMi[7].rotation();
        for (int i = 0; i < 3; ++i){
            armParam.des_translation[i] = robotarm.data.oMi[7].translation()(i);
            armParam.des_rotation[i] = rot2rpy(P_rotation_ref)[i];
            std::cout << "Initial Translation & Rotation: " << robotarm.data.oMi[7].translation()(i) << "   "
            << rot2rpy(P_rotation_ref)[i] << std::endl;
        }
    }
        
    userMoveArm();
#endif
    // getTestData();
}

void ArmController::getQDes(){
    robotarm.InverseKinematic(armdata.arm_q, Pdes_ref, q_end, P_rotation_ref);
    std::cout << "EE current position: " << std::endl << robotarm.data.oMi[7] << "    "
    << "EE desired position: " << Pdes_ref.transpose() << std::endl;
}

void ArmController::movearm(){
    trajectory.getJointCmd(qdes_arm, dqdes_arm);
    std::cout << "Current Time: " << trajectory._T << std::endl;


    for (int i = 0; i < 6; i++){
        armcmd.full_q[i] = qdes_arm[i];
        armcmd.full_dq[i] = dqdes_arm[i];
    }
    armcmd.full_q[6] = 0;
    armcmd.full_dq[6] = 0;
    // std::cout << "end! " << std::endl;
}

void ArmController::userMoveArm(){
    Pdes_ref << armParam.des_translation[0], armParam.des_translation[1], armParam.des_translation[2];
    Prot_ref << armParam.des_rotation[0], armParam.des_rotation[1], armParam.des_rotation[2];
    std::cout << "User Input POS: " << Pdes_ref.transpose() << std::endl
    << "User INput ROT: " << Prot_ref.transpose() << std::endl;
    P_rotation_ref = rpy2rot(Prot_ref);
    gripper_q_ref = armParam.gripper_des_q;

    robotarm.InverseKinematic(armdata.arm_q, Pdes_ref, qdes_arm, P_rotation_ref);

    for (int i = 0; i < 6; i++){
        armcmd.full_q[i] = qdes_arm[i];
        // armcmd.full_dq[i] = dqdes_arm[i];
    }
    armcmd.full_q[6] = gripper_q_ref;
    // armcmd.full_q[6] = 0;
    // // armcmd.full_dq[6] = 0;
}

void ArmController::getTestData(){
    double target_qpos3[7] = {0.5, -1, -2.093, 0.5, -0.4, 1, 0};
    for (int i = 0; i < 7; i++){
        armcmd.full_q[i] = target_qpos3[i];
    }
    pinocchio::forwardKinematics(robotarm.model, robotarm.data, armcmd.full_q);
    std::cout << "Joint 7 translation: " << robotarm.data.oMi[7].translation() << std::endl;
    std::cout << "Joint 7 rotation:" << rot2rpy(robotarm.data.oMi[7].rotation()) << std::endl;
}

void ArmController::updateState(){
    for (int i = 0; i < 7; i++){
        armdata.arm_q(i) = _motorState->qpos(i);
        armdata.arm_dq(i) = _motorState->qvel(i);
    }
}

void ArmController::computeKinematicsDynamics(){
    armdata.J.setZero();
    armdata.dJ.setZero();

    robotarm.ComputeAllTerm(armdata.arm_q, armdata.arm_dq);
    robotarm.ComputeCoriolisMatrix(armdata.arm_q, armdata.arm_dq);
    robotarm.ComputeGeneralizedGravity(armdata.arm_dq);

    armdata.M = robotarm.data.M;
    armdata.C = robotarm.data.C;
    armdata.G = robotarm.data.g;
    // std::cout << "M: " << armdata.M << std::endl;
    // std::cout << "C: " << armdata.C << std::endl;
    // std::cout << "g: " << armdata.G << std::endl;
}

void ArmController::updateCommand(){
    // set limit for joint torque
    // for (int i = 0; i < 7; i++){
    //     if(abs(armcmd.full_tau(i)) > armParam.torquelimit(i)){
    //         armcmd.full_tau(i) = armParam.torquelimit(i) * abs(armcmd.full_tau(i)) / armcmd.full_tau(i);
    //     }
    // }

    // if (armcmd.full_tau.hasNaN()){
    //     armcmd.full_tau = laststate.full_tau_last;
    // }

    // for (int i = 0; i < 7; i++){
    //     _motorCmd->tau_ff[i] = armcmd.full_tau[i];
    // }
    for (int i = 0; i < 7; i++){
        _motorCmd->qpos_des[i] = armcmd.full_q[i];
    }
}

void ArmController::updateBuffer(){
    laststate.full_tau_last = armcmd.full_tau;
}

Eigen::Vector3d ArmController::rot2rpy(Eigen::Matrix3d R)
{
    Eigen::Vector3d rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = atan2(-R(2, 0), std::pow(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2), 0.5));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}

Eigen::Matrix3d ArmController::rpy2rot(Eigen::Vector3d euler){
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());
    return rot;
}

ArmController::~ArmController(){
    _sharedMemory().ctrl_attached = false;
    _sharedMemory.detach();
    delete _motorCmd;
    delete _motorState;
}