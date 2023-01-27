#include "Team971FFU.h"
#include <cmath>
#include "Constants.h"
#include <Eigen/LU>

Team971FFU::Team971FFU() {
    
}

std::array<Eigen::Matrix<double, 2, 2>, 4> Team971FFU::NormalizedMatricesForState(Eigen::Matrix<double, 4, 1> X) {
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

std::array<Eigen::Matrix<double, 2, 2>, 4> Team971FFU::MatricesForState(Eigen::Matrix<double, 4, 1> X) {
    auto matrices = NormalizedMatricesForState(X);
    matrices.at(2)(1, 0) *= X(1, 0);
    matrices.at(2)(0, 1) *= X(3, 0);
    return matrices;
}

Eigen::Matrix<double, 2, 1> Team971FFU::ff_u(Eigen::Matrix<double, 4, 1> X, Eigen::Matrix<double, 2, 1> omega_t, Eigen::Matrix<double, 2, 1> alpha_t) {
    auto matrices = MatricesForState(X);

    // inverse K3
    return matrices.at(2).inverse() * (matrices.at(0) * alpha_t + matrices.at(1) * omega_t + matrices.at(3) * omega_t);
}

std::array<double, 2> Team971FFU::getffu(double thetaArm1, double thetaArm2, double omegaArm1, double omegaArm2, double alphaArm1, double alphaArm2) {
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