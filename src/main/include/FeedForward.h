#pragma once

//#include <Eigen/src/Core/Matrix.h>
//#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Core>
#include "TrajectoryManager.h"

class FeedForward {
public:
    FeedForward();
    std::array<double, 2> getffu(ArmTrajectoryPoint trajPoint);
    /**
     * This function needs the second arm position, velocity, and acceleration relative to the ground, NOT relative to the frist arm.
    */
    std::array<double, 2> getffu(double thetaArm1, double thetaArm2, double omegaArm1, double omegaArm2, double alphaArm1, double alphaArm2);
private:
    std::array<Eigen::Matrix<double, 2, 2>, 4> NormalizedMatricesForState(Eigen::Matrix<double, 4, 1> X);
    std::array<Eigen::Matrix<double, 2, 2>, 4> MatricesForState(Eigen::Matrix<double, 4, 1> X);
    Eigen::Matrix<double, 2, 1> ff_u(Eigen::Matrix<double, 4, 1> X, Eigen::Matrix<double, 2, 1> omega_t, Eigen::Matrix<double, 2, 1> alpha_t);

    bool configFeedForward = true;
    bool m_zeroOmegaAlpha = false;

    double m_l1 = FFUConstants::l1;
    double m_l2 = FFUConstants::l2;
    
    double m_m1 = FFUConstants::m1;
    double m_m2 = FFUConstants::m2;

    double m_r1 = FFUConstants::r1;
    double m_r2 = FFUConstants::r2;

    double m_I1 = FFUConstants::I1;
    double m_I2 = FFUConstants::I2;

    double m_G1 = FFUConstants::G1;
    double m_G2 = FFUConstants::G2;

    double m_stall_torque = FFUConstants::stall_torque;
    double m_free_speed = FFUConstants::free_speed;
    double m_stall_current = FFUConstants::stall_current;
    double m_R = 12.0 / m_stall_current;

    double m_Kv = m_free_speed / 12.0;
    double m_Kt = m_stall_torque / m_stall_current;

    double m_alpha = m_I1 + m_r1 * m_r1 * m_m1 + m_l1 * m_l1 * m_m2;
    double m_beta = m_l1 * m_r2 * m_m2;
    double m_gamma = m_I2 + m_r2 * m_r2 * m_m2;

    double m_kNumDistalMotors = 2.0;

    Eigen::Matrix<double, 2, 2> m_K3 {
        {m_G1 * m_Kt / m_R, 0.0},
        {0.0, m_G2 * m_kNumDistalMotors * m_Kt / m_R}
    };
    Eigen::Matrix<double, 2, 2> m_K4 {
        {m_G1 * m_G1 * m_Kt / (m_Kv * m_R), 0.0},
        {0.0, m_G2 * m_G2 * m_Kt * m_kNumDistalMotors / (m_Kv * m_R)}
    };

    double m_kG1 = FFUConstants::kG1;
    double m_kG2 = FFUConstants::kG2;
    double m_kGOtherArm = FFUConstants::kGOtherArm;

    double m_arm_1_trajectory_time_multiplier = FFUConstants::arm_1_trajectory_time_multiplier;
    double m_arm_2_trajectory_time_multiplier = FFUConstants::arm_2_trajectory_time_multiplier;
};