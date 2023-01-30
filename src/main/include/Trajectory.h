#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <tuple>
#include "Constants.h"

typedef std::tuple<std::array<units::radian_t, FFUConstants::trajectory_size>, 
    std::array<units::radians_per_second_t, FFUConstants::trajectory_size>, 
    std::array<units::radians_per_second_squared_t, FFUConstants::trajectory_size>>
    ArmTrajectoryData;

class Trajectory {
public:
    static ArmTrajectoryData calcTraj(units::radian_t start_theta, units::radian_t end_theta, units::second_t time);
private:
    static std::array<units::radian_t, FFUConstants::trajectory_size> get_thetas(units::radians_per_second_t h, double A, units::second_t time, units::radian_t start_theta);
    static std::array<units::radians_per_second_t, FFUConstants::trajectory_size> get_omegas(units::radians_per_second_t h, double A, units::second_t time);
    static std::array<units::radians_per_second_squared_t, FFUConstants::trajectory_size> get_alphas(units::radians_per_second_t h, double A, units::second_t time);
};