#include "FeedForward.h"
#include <cmath>
#include "Constants.h"
#include <Eigen/LU>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

FeedForward::FeedForward() {
    // put vals to smartdashboard
    frc::SmartDashboard::PutNumber("l1", m_l1);
    frc::SmartDashboard::PutNumber("l2", m_l2);
    frc::SmartDashboard::PutNumber("m1", m_m1);
    frc::SmartDashboard::PutNumber("m2", m_m2);
    frc::SmartDashboard::PutNumber("r1", m_r1);
    frc::SmartDashboard::PutNumber("r2", m_r2);
    frc::SmartDashboard::PutNumber("I1", m_I1);
    frc::SmartDashboard::PutNumber("I2", m_I2);
    frc::SmartDashboard::PutNumber("G1", m_G1);
    frc::SmartDashboard::PutNumber("G2", m_G2);
    frc::SmartDashboard::PutNumber("stall_torque", m_stall_torque);
    frc::SmartDashboard::PutNumber("stall_current", m_stall_current);
    frc::SmartDashboard::PutNumber("free_speed", m_free_speed);
    frc::SmartDashboard::PutNumber("kG1", m_kG1);
    frc::SmartDashboard::PutNumber("kG2", m_kG2);
    frc::SmartDashboard::PutNumber("kGOtherArm", m_kGOtherArm);
    frc::SmartDashboard::PutNumber("arm_1_trajectory_multiplier", m_arm_1_trajectory_time_multiplier);
    frc::SmartDashboard::PutNumber("arm_2_trajectory_multiplier", m_arm_2_trajectory_time_multiplier);
    frc::SmartDashboard::PutNumber("feedforward zero omega alpha", m_zeroOmegaAlpha);
}

/**
 * Calculates normalized feedforward matrices
 * 
 * @param X 4x1 vecotor with (theta1, omega1, theta2, omega2) desired state
 * @return Returns normalized versions of feedforward matrices
 */
std::array<Eigen::Matrix<double, 2, 2>, 4> FeedForward::NormalizedMatricesForState(Eigen::Matrix<double, 4, 1> X) {
    double s = std::sin(X(0, 0) - X(2, 0));
    double c = std::cos(X(0, 0) - X(2, 0));

    Eigen::Matrix<double, 2, 2> K1 {
        {m_alpha, c * m_beta},
        {c * m_beta, m_gamma}
    };

    Eigen::Matrix<double, 2, 2> K2 {
        {0.0, s * m_beta},
        {-s * m_beta, 0.0}
    };

    return {K1, K2, m_K3, m_K4};
}

std::array<double, 2> FeedForward::getffu(ArmTrajectoryPoint trajPoint) {
    return FeedForward::getffu(
        std::get<0>(trajPoint).first.to<double>(),
        std::get<0>(trajPoint).second.to<double>(),
        std::get<1>(trajPoint).first.to<double>(),
        std::get<1>(trajPoint).second.to<double>(),
        std::get<2>(trajPoint).first.to<double>(),
        std::get<2>(trajPoint).second.to<double>());
}

/**
 * Un-normalizes K3 matrix to be specialized for desired state 
 * 
 * @param X 4x1 vecotor with (theta1, omega1, theta2, omega2) desired state
 * @return ready to use feedforward matrices
 */
std::array<Eigen::Matrix<double, 2, 2>, 4> FeedForward::MatricesForState(Eigen::Matrix<double, 4, 1> X) {
    auto matrices = NormalizedMatricesForState(X);
    matrices.at(2)(1, 0) *= X(1, 0);
    matrices.at(2)(0, 1) *= X(3, 0);
    return matrices;
}

/**
 * Calculates voltages for top and bottom arm segments.
 * 
 * @param X the desired state
 * @param omega_t the desired angular velocity
 * @param alpha_t the desired angualr acceleration
 * @return 2d vector with voltages for proximal and distal pivot motors
 */
Eigen::Matrix<double, 2, 1> FeedForward::ff_u(Eigen::Matrix<double, 4, 1> X, Eigen::Matrix<double, 2, 1> omega_t, Eigen::Matrix<double, 2, 1> alpha_t) {
    auto matrices = MatricesForState(X);

    Eigen::Matrix<double, 2, 1> theta_real_cos {
        {std::cos(X(0, 0))},
        {std::cos(X(2, 0) - X(0, 0))}
    };

    Eigen::Matrix<double, 2, 2> KGravity {
        {m_kG1, m_kG2},
        {0.0, m_kG2}
    };

    auto torque = matrices.at(0) * alpha_t + matrices.at(1) * omega_t + matrices.at(3) * omega_t + KGravity * theta_real_cos; 

    // inverse K3
    return matrices.at(2).inverse() * torque;
}

/**
 * Easier to call feedforward calculation. 
 * 
 * @param thetaArm1 
 * @param thetaArm2 
 * @param omegaArm1 
 * @param omegaArm2 
 * @param alphaArm1 
 * @param alphaArm2 
 * @return array of 2 elements, the calculated voltages for proximal and distal pivot motors
 */
std::array<double, 2> FeedForward::getffu(double thetaArm1, double thetaArm2, double omegaArm1, double omegaArm2, double alphaArm1, double alphaArm2) {
    // get values from smartdashboard
    m_l1 = frc::SmartDashboard::GetNumber("l1", m_l1);
    m_l2 = frc::SmartDashboard::GetNumber("l2", m_l2);
    m_m1 = frc::SmartDashboard::GetNumber("m1", m_m1);
    m_m2 = frc::SmartDashboard::GetNumber("m2", m_m2);
    m_r1 = frc::SmartDashboard::GetNumber("r1", m_r1);
    m_r2 = frc::SmartDashboard::GetNumber("r2", m_r2);
    m_I1 = frc::SmartDashboard::GetNumber("I1", m_I1);
    m_I2 = frc::SmartDashboard::GetNumber("I2", m_I2);
    m_G1 = frc::SmartDashboard::GetNumber("G1", m_G1);
    m_G2 = frc::SmartDashboard::GetNumber("G2", m_G2);
    m_stall_torque = frc::SmartDashboard::GetNumber("stall_torque", m_stall_torque);
    m_stall_current = frc::SmartDashboard::GetNumber("stall_current", m_stall_current);
    m_free_speed = frc::SmartDashboard::GetNumber("free_speed", m_free_speed);
    m_kGOtherArm = frc::SmartDashboard::GetNumber("kGOtherArm", m_kGOtherArm);
    m_kG1 = frc::SmartDashboard::GetNumber("kG1", m_kG1);
    m_kG2 = frc::SmartDashboard::GetNumber("kG2", m_kG2);
    m_arm_1_trajectory_time_multiplier = frc::SmartDashboard::GetNumber("arm_1_trajectory_multiplier", m_arm_1_trajectory_time_multiplier);
    m_arm_2_trajectory_time_multiplier = frc::SmartDashboard::GetNumber("arm_2_trajectory_multiplier", m_arm_2_trajectory_time_multiplier);
    m_zeroOmegaAlpha = frc::SmartDashboard::GetBoolean("feedforward zero omega alpha", m_zeroOmegaAlpha);

    if (m_zeroOmegaAlpha) {
        omegaArm1 = 0.0;
        omegaArm2 = 0.0;
        alphaArm1 = 0.0;
        alphaArm2 = 0.0;
    }

    m_R = 12.0 / m_stall_current;
    m_Kt = m_stall_torque / m_stall_current;
    m_Kv = m_free_speed / 12.0;

    m_K3 = Eigen::Matrix<double, 2, 2> {
        {m_G1 * m_Kt / m_R, 0.0},
        {0.0, m_G2 * m_kNumDistalMotors * m_Kt / m_R}
    };

    m_K4 = Eigen::Matrix<double, 2, 2> {
        {m_G1 * m_G1 * m_Kt / (m_Kv * m_R), 0.0},
        {0.0, m_G2 * m_G2 * m_Kt * m_kNumDistalMotors / (m_Kv * m_R)}
    };
    

    Eigen::Matrix<double, 4, 1> X {
        {thetaArm1},
        {omegaArm1},
        {thetaArm1},
        {omegaArm2}
    };

    Eigen::Matrix<double, 2, 1> omega_t {
        {omegaArm1},
        {omegaArm2}
    };

    Eigen::Matrix<double, 2, 1> alpha_t {
        {alphaArm1},
        {alphaArm2}
    };

    Eigen::Matrix<double, 2, 1> ret = ff_u(X, omega_t, alpha_t);

    //double gravityAddArm1 = m_kG1 * std::cos(thetaArm1) + ret(1, 0) * m_kGOtherArm;
    //double gravityAddArm2 = m_kG2 * std::cos(thetaArm2);

    return {ret(0, 0), ret(1, 0)};
}