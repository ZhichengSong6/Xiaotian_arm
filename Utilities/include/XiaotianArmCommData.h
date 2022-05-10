#ifndef MININEZHA_COMM_DATA_H
#define MININEZHA_COMM_DATA_H

#include "cppTypes.h"

struct IMUData {
    Eigen::Vector4d quat;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
};

struct JointsState {
    Vec7 qpos;  // left leg (hip knee wheel), right leg, arm
    Vec7 qvel;
    Vec7 tau;
};

struct MeasuredState {
    IMUData imuData; // [w x y z]
    JointsState jointsState;
};

struct JointsCmd {
    Vec7 Kp = Vec7::Zero();
    Vec7 Kd = Vec7::Zero();
    Vec7 qpos_des = Vec7::Zero();
    Vec7 qvel_des = Vec7::Zero();
    Vec7 tau_ff = Vec7::Zero();
};

#endif //MININEZHA_COMM_DATA_H