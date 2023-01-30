#pragma once

#include <math.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>
#include <units/angular_velocity.h>
#include "string"
#include "frc/I2C.h"
#include <frc/Filesystem.h>
#include <frc/controller/RamseteController.h>
#include <Eigen/Core>

using namespace std;

#define M_PI 3.14159265358979323846 //for simulation

namespace GeneralConstants
{
    const int TICKS_PER_ROTATION = 2048;
    const double TICKS_PER_RADIAN = TICKS_PER_ROTATION/(2*M_PI);
}

namespace ArmConstants{
    const int BASE_MOTOR_ID = 0;
    const int TOP_MOTOR_ID = 0;

    const double BASE_ARM_LENGTH = 0.0;
    const double TOP_ARM_LENGTH = 0.0;
    const double PIVOT_HEIGHT = 0.0;

    const double BASE_OFFSET = 0.0; //Radians
    const double TOP_OFFSET = 0.0; //Radians

    const double BASE_PID[3] = {0.0, 0.0, 0.0};
    const double TOP_PID[3] = {0.0, 0.0, 0.0};
}

namespace AprilTagsConstants {
    const units::meter_t FIELD_LENGTH = 54_ft + 3.25_in;
    const units::meter_t FIELD_WIDTH = 26_ft + 3.25_in;
    const units::meter_t GRID_APRILTAG_X = 3_ft + 1_in;
    const units::meter_t GRID_APRILTAG_LOW_Y = 3_ft + 6_in;
    const units::meter_t GRID_APRILTAG_MID_Y = 9_ft;
    const units::meter_t GRID_APRILTAG_TOP_Y = 14_ft + 6_in;
    const units::meter_t DOUBLE_SUBSTATION_APRILTAG_X = 1_ft + 2_in;
    const units::meter_t DOUBLE_SUBSTATION_APRILTAG_Y = 22_ft + 0.3125_in;
}

namespace FFUConstants {
    // TODO tune these constants
    const double l1 = 1;
    const double l2 = 1;
    
    const double m1 = 1;
    const double m2 = 1;

    const double I1 = 1;
    const double I2 = 1;

    const double r1 = 1;
    const double r2 = 1;

    const double G1 = 1;
    const double G2 = 1;

    const double stall_torque = 1;
    const double free_speed = 1;
    const double stall_current = 1;
    const double R = 12.0 / stall_current;

    const double Kv = free_speed / 12.0;
    const double Kt = stall_torque / stall_current;

    const double alpha = I1 + r1 * r1 * m1 + l1 * l1 * m2;
    const double beta = l1 * r2 * m2;
    const double gamma = I2 + r2 * r2 * m2;

    const double kNumDistalMotors = 2.0;

    const Eigen::Matrix<double, 2, 2> K3 {
        {G1 * Kt / R, 0.0},
        {0.0, G2 * kNumDistalMotors * Kt / R}
    };
    const Eigen::Matrix<double, 2, 2> K4 {
        {G1 * G1 * Kt / (Kv * R), 0.0},
        {0.0, G2 * G2 * Kt * kNumDistalMotors / (Kv * R)}
    };

    const double kG1 = 1;
    const double kG2 = 1;
    const double kGOtherArm = 1;

    const int trajectory_size = 1000;

    const double arm_1_trajectory_time_multiplier = 1;
    const double arm_2_trajectory_time_multiplier = 1;
}