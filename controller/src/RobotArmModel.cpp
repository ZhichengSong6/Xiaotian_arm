#include "RobotArmModel.h"

RobotArmModel::RobotArmModel()
{
    // load the model from urdf file
    pinocchio::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << "load succeefully! " << std::endl;
    data = Data(model);
    q = pinocchio::neutral(model);
    v = Eigen::VectorXd::Zero(model.nv);
    J = Eigen::MatrixXd::Zero(6, model.nv);
    dJ = Eigen::MatrixXd::Zero(6, model.nv);

    eps = 1e-2;
    IT_MAX = 500;
    DT = 1e-1;
    damp = 1e-6;
}

void RobotArmModel::ComputeAllTerm(Eigen::VectorXd q_now, Eigen::VectorXd v_now)
{
    q = q_now;
    v = v_now;
    pinocchio::computeAllTerms(model, data, q, v);
}

void RobotArmModel::ComputeCoriolisMatrix(Eigen::VectorXd q_now, Eigen::VectorXd v_now)
{
    q = q_now;
    v = v_now;
    pinocchio::computeCoriolisMatrix(model, data, q, v);
}

void RobotArmModel::ComputeGeneralizedGravity(Eigen::VectorXd q_now)
{
    q = q_now;
    pinocchio::computeGeneralizedGravity(model, data, q);
}

// Eigen::Vector3d RobotArmModel::getJointTranslation()
// {
//     return (data.oMi[6].translation());
// }

Eigen::Vector3d RobotArmModel::getJointTranslation_LOCAL(Eigen::VectorXd q_now)
{
    q = q_now;
    pinocchio::forwardKinematics(model, data, q);
    return (data.oMi[7].translation());
}

// Eigen::Matrix3d RobotArmModel::getJointRotation()
// {
//     return (data.oMi[6].rotation());
// }

Data::Matrix6x RobotArmModel::getJointJacobian()
{
    Data::Matrix6x Je;
    pinocchio::getJointJacobian(model, data, 6, LOCAL_WORLD_ALIGNED, Je);
    return Je;
}

Data::Matrix6x RobotArmModel::getJacobDerivative()
{
    Data::Matrix6x dJe;
    pinocchio::getJointJacobianTimeVariation(model, data, 6, LOCAL_WORLD_ALIGNED, dJe);
    return dJe;
}

void RobotArmModel::InverseKinematic(Eigen::VectorXd q_now, Eigen::Vector3d Pdes, Eigen::VectorXd &qdes, Eigen::Matrix3d rot)
{
    JOINT_ID = 7;
    v.setZero();
    q.setZero();
    q = q_now;

    pinocchio::SE3 oMdes(rot, Pdes);
    for (int i = 0;; i++)
    {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        err = pinocchio::log6(dMi).toVector();
        if (err.norm() < eps)
        {
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            success = false;
            std::cout << "conergence failed" << std::endl;
            break;
        }
        pinocchio::computeJointJacobian(model, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        // Eigen::Matrix3d JJt;
        // Ji = J.block<3, 7>(0, 0);
        // JJt.noalias() = Ji * Ji.transpose();
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, v * DT);
    }

    if (success)
    {
        for (int i = 0; i < model.nq; i++)
        {
            q(i) = fmod(q(i), (2 * M_PI));
        //     // if (q_fix(i) >= M_PI && q_fix(i) <= 2 * M_PI)
        //     // {
        //     //     q_fix(i) = q_fix(i) - 2 * M_PI;
        //     // }
        //     // if (q_fix(i) <= -M_PI && q_fix(i) >= -2 * M_PI)
        //     // {
        //     //     q_fix(i) = q_fix(i) + 2 * M_PI;
        //     // }
        }
        // std::cout << "success!" << std::endl;
        qdes = q.block<6, 1>(0, 0);
    }
    else
    {
        qdes = q_now;
        std::cout << "Joint id:" << JOINT_ID << std::endl
                  << " Warning: the iterative algorithm has not reached convergence to the desired precision"
                  << std::endl;
    }
}

int RobotArmModel::GetNq()
{
    return model.nq;
}

int RobotArmModel::GetNv()
{
    return model.nv;
}