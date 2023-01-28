#pragma once

//#include <Eigen/src/Core/Matrix.h>
//#include <Eigen/src/Core/Matrix.h>
#include <Eigen/Core>

class FeedForward {
public:
    FeedForward();
    /**
     * This function needs the second arm position, velocity, and acceleration relative to the ground, NOT relative to the frist arm.
    */
    static std::array<double, 2> getffu(double thetaArm1, double thetaArm2, double omegaArm1, double omegaArm2, double alphaArm1, double alphaArm2);
private:
    static std::array<Eigen::Matrix<double, 2, 2>, 4> NormalizedMatricesForState(Eigen::Matrix<double, 4, 1> X);
    static std::array<Eigen::Matrix<double, 2, 2>, 4> MatricesForState(Eigen::Matrix<double, 4, 1> X);
    static Eigen::Matrix<double, 2, 1> ff_u(Eigen::Matrix<double, 4, 1> X, Eigen::Matrix<double, 2, 1> omega_t, Eigen::Matrix<double, 2, 1> alpha_t);
};