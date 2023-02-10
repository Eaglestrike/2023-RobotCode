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

namespace GeneralConstants{
    const double TICKS_PER_ROTATION = 4096.0;
    const double TICKS_PER_RADIAN = TICKS_PER_ROTATION/(2*M_PI);
}

namespace InputConstants{
    const int XBOX_PORT = 2;
    const int XBOX_LJOY_X = 0;
    const int XBOX_LJOY_Y = 1;
    const int XBOX_LTRIGGER = 2;
    const int XBOX_RTRIGGER = 3;
    const int XBOX_RJOY_X = 4;
    const int XBOX_RJOY_Y = 5;
}

namespace ArmConstants{
    const int BASE_MOTOR_ID = 20;
    const int TOP_MOTOR_ID = 14;

    const double BASE_ARM_LENGTH = 31.0 * 0.0254;
    const double TOP_ARM_LENGTH = 31.0 * 0.0254;
    const double PIVOT_HEIGHT = 32.0 * 0.0254;

    const double BASE_OFFSET = 0.0; //Radians
    const double TOP_OFFSET = 0.0; //Radians

    const double BASE_PID[3] = {20.0, 0.0, 0.0}; //Error in radians
    const double BASE_KGRAVITY = 3.0;
    const double TOP_PID[3] = {15.0, 0.0, 0.0}; //Error in radians
    const double TOP_KGRAVITY = 2.0;

    const double MAX_VOLTS = 5;

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
    const double l1 = 31.0 * 0.0254;
    const double l2 = 31.0 * 0.0254;
    
    const double m1 = 0.381;
    const double m2 = 0.381;

    const double r1 = 31.0 / 2 * 0.0254;
    const double r2 = 31.0 / 2 * 0.0254;

    const double I1 = r1 * r1 * m1;
    const double I2 = r2 * r2 * m2;

    const double G1 = 100.0;
    const double G2 = 50.0;

    const double stall_torque = 4.69;
    const double free_speed = (6380.0 / 60.0) * 2 * M_PI;
    const double stall_current = 257;
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

    const double kG1 = m1 * 9.81 * r1;
    const double kG2 = m2 * 9.81 * r2;
    const double kGOtherArm = 1;

    const int trajectory_size = 1000;

    const double arm_1_trajectory_time_multiplier = 1;
    const double arm_2_trajectory_time_multiplier = 1;
}