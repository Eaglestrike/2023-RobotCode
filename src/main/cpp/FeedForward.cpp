#include "FeedForward.h"
#include <cmath>
#include "Constants.h"
#include <Eigen/LU>

FeedForward::FeedForward() {
    
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
        {FFUConstants::alpha, c * FFUConstants::beta},
        {c * FFUConstants::beta, FFUConstants::gamma}
    };

    Eigen::Matrix<double, 2, 2> K2 {
        {0.0, s * FFUConstants::beta},
        {-s * FFUConstants::beta, 0.0}
    };

    return {K1, K2, FFUConstants::K3, FFUConstants::K4};
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

    Eigen::Matrix<double, 2, 1> theta_real {
        {X(0, 0)},
        {X(2, 0) - X(0, 0)}
    };

    Eigen::Matrix<double, 2, 2> KGravity {
        {FFUConstants::kG1, FFUConstants::kGOtherArm + FFUConstants::kG2},
        {0.0, FFUConstants::kG2}
    };

    auto torque = matrices.at(0) * alpha_t + matrices.at(1) * omega_t + matrices.at(3) * omega_t + KGravity * theta_real;

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

    double gravityAddArm1 = FFUConstants::kG1 * std::cos(thetaArm1) + ret(1, 0) * FFUConstants::kGOtherArm;
    double gravityAddArm2 = FFUConstants::kG2 * std::cos(thetaArm2);

    return {ret(0, 0) + gravityAddArm1, ret(1, 0) + gravityAddArm2};
}