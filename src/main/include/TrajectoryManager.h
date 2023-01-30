#pragma once

#include <tuple>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <frc/Timer.h>
#include "Trajectory.h"

typedef std::tuple<std::pair<units::radian_t, units::radian_t>,
    std::pair<units::radians_per_second_t, units::radians_per_second_t>,
    std::pair<units::radians_per_second_squared_t, units::radians_per_second_squared_t>>
    ArmTrajectoryPoint;

class TrajectoryManager {
public:
    ArmTrajectoryPoint get_traj(
        units::radian_t arm1Curr, units::radian_t arm1Target,
        units::radian_t arm2Curr, units::radian_t arm2Target
    );
private:
    void gen_new_traj(
        units::radian_t arm1Curr, units::radian_t arm1Target,
        units::radian_t arm2Curr, units::radian_t arm2Target
    );

    units::second_t start_time = frc::Timer::GetFPGATimestamp();
    units::second_t elapsed_time;
    units::second_t arm_1_trajectory_time;
    units::second_t arm_2_trajectory_time;
    ArmTrajectoryData arm_1_trajectory;
    ArmTrajectoryData arm_2_trajectory;
    units::radian_t old_arm_1_target;
    units::radian_t old_arm_2_target;
    units::radian_t original_arm_1_angle;

    bool firstRun;
};