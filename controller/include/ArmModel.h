#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <iostream>
#include "CtrlData.h"
#include "Math/orientation_tools.h"

using namespace pinocchio;

class ArmModel
{
public:
    ArmModel(/* args */);
    ~ArmModel() = default;
    void InverseKinematic(Eigen::Matrix<double, 6, 1>, Eigen::Vector3d Pdes, Eigen::Matrix<double, 6, 1> &qdes, Eigen::Matrix<double, 6, 1> &dqdes);
    Eigen::Vector3d getJointTranslation(Eigen::Matrix<double, 6, 1> q_now);
    Data::Matrix6x getJointJacobian(Eigen::Matrix<double, 6, 1> q_now);
    Data::Matrix6x getJacobDerivative(Eigen::Matrix<double, 6, 1> q_now, Eigen::Matrix<double, 6, 1> dq_now);

private:
    Model model;
    Data data;

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
    const std::string urdf_filename = std::string("../../robot_model/ARM_XML/ARM.urdf");


};

