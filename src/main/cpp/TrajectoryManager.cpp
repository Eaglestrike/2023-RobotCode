#include "TrajectoryManager.h"

ArmTrajectoryPoint TrajectoryManager::get_traj(units::radian_t arm1Curr, units::radian_t arm1Target,
units::radian_t arm2Curr, units::radian_t arm2Target) {
    arm2Curr += arm1Curr;
    arm2Target += original_arm_1_angle;
    if (firstRun || arm1Target != old_arm_1_target || arm2Target != old_arm_2_target) {
        gen_new_traj(arm1Curr, arm1Target, arm2Curr, arm2Target);
    }

    int index_arm_1 = FFUConstants::trajectory_size * elapsed_time / arm_1_trajectory_time;
    int index_arm_2 = FFUConstants::trajectory_size * elapsed_time / arm_2_trajectory_time;

    units::radian_t arm_1_theta;
    units::radian_t arm_2_theta;
    units::radians_per_second_t arm_1_omega;
    units::radians_per_second_t arm_2_omega;
    units::radians_per_second_squared_t arm_1_alpha;
    units::radians_per_second_squared_t arm_2_alpha;

    if (elapsed_time < arm_1_trajectory_time) {
        arm_1_theta = std::get<0>(arm_1_trajectory)[index_arm_1];
        arm_1_omega = std::get<1>(arm_1_trajectory)[index_arm_1];
        arm_1_alpha = std::get<2>(arm_1_trajectory)[index_arm_1];
    } else {
        arm_1_theta = arm1Target;
        arm_1_omega = 0_rad_per_s;
        arm_1_alpha = 0_rad_per_s_sq;
    }

    if (elapsed_time < arm_2_trajectory_time) {
        arm_2_theta = std::get<0>(arm_2_trajectory)[index_arm_2];
        arm_2_omega = std::get<1>(arm_2_trajectory)[index_arm_2];
        arm_2_alpha = std::get<2>(arm_2_trajectory)[index_arm_2];
    } else {
        arm_2_theta = arm2Target;
        arm_2_omega = 0_rad_per_s;
        arm_2_alpha = 0_rad_per_s_sq;
    }

    auto ret = std::make_tuple(
        std::make_pair(arm_1_theta, arm_2_theta),
        std::make_pair(arm_1_omega, arm_2_omega),
        std::make_pair(arm_1_alpha, arm_2_alpha)
    );

    elapsed_time = frc::Timer::GetFPGATimestamp() - start_time;
    old_arm_1_target = arm1Target;
    old_arm_2_target = arm2Target;

    return ret;
}

void TrajectoryManager::gen_new_traj(units::radian_t arm1Curr, units::radian_t arm1Target,
units::radian_t arm2Curr, units::radian_t arm2Target) {
    start_time = frc::Timer::GetFPGATimestamp();
    elapsed_time = 0_s;
    arm_1_trajectory_time = 0_s;
    arm_2_trajectory_time = 0_s;
    arm_1_trajectory = Trajectory::calcTraj(arm1Curr, arm1Target, arm_1_trajectory_time);
    arm_2_trajectory = Trajectory::calcTraj(arm2Curr, arm2Target, arm_2_trajectory_time);
    firstRun = false;
    original_arm_1_angle = arm1Curr;
}