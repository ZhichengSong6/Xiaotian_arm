#ifndef XIAOTIANARM_MODEL_H
#define XIAOTIANARM_MODEL_H

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <iostream>
#include "Math/orientation_tools.h"

using namespace pinocchio;

class RobotArmModel
{

public:
    RobotArmModel();
    ~RobotArmModel() = default;
    void InverseKinematic(Eigen::VectorXd q_now, Eigen::Vector3d Pdes, Eigen::VectorXd &qdes, Eigen::Matrix3d rot);
    
    Eigen::Vector3d getJointTranslation();
    Eigen::Vector3d getJointTranslation_LOCAL(Eigen::VectorXd q_now);
    Eigen::Matrix3d getJointRotation();
    Data::Matrix6x getJointJacobian();
    Data::Matrix6x getJacobDerivative();
    
    int GetNv();
    int GetNq();
    void ComputeAllTerm(Eigen::VectorXd q, Eigen::VectorXd v);
    void ComputeCoriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd v);
    void ComputeGeneralizedGravity(Eigen::VectorXd q);
    
    Data data;
    Model model;

private:

    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::MatrixXd J;
    Eigen::MatrixXd dJ;
    Eigen::MatrixXd Ji;
    Eigen::Matrix<double, 6, 1> err;

    int JOINT_ID;
    double eps;
    double damp;
    int IT_MAX;
    double DT;
    double ydes;
    bool success = false;
    const std::string urdf_filename = std::string("../../robot_model/ARM_XML/Xiaotian_Arm_v3.urdf");

};

#endif