#pragma once

#define _USE_MATH_DEFINES
#include <math.h>
#include "string"
#include "frc/Filesystem.h"

using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510582097494459230
#endif

namespace GeneralConstants
{
    const double g = 9.81;

    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE / STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * M_PI) / 60) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);

    const double CONE_M = 0.652;

}

namespace FieldConstants
{
    const double FIELD_WIDTH = 8.2296;
    const double FIELD_LENGTH = 16.4592;
    // const double LEFT_TAG_X = 1.05283; //41.45 in
    // const double MIDDLE_TAG_X = 2.72923; //107.45 in
    // const double RIGHT_TAG_X = 4.40563; //173.45;

    /**
     * Tag Positions in the form of
     * TAG_XY[TagID -1] = {x, y}
     */
    const double TAG_XY[8][2] = {
        {15.513558, 1.071626},
        {15.513558, 2.748026},
        {15.513558, 4.424426},
        {16.178784, 6.749796},
        {0.36195, 6.749796},
        {1.02743, 4.424426},
        {1.02743, 2.748026},
        {1.02743, 1.071626}};

    const double BOTTOM_CONE_Y = 0.5128;
    const double TOP_CONE_Y = 4.983;
    const double BOTTOM_CUBE_Y = 1.0716;
    const double MID_CUBE_Y = 2.748026;
    const double TOP_CUBE_Y = 4.4244;

    const double TOP_MIDDLE_CONE_Y = 3.306826;
    const double BOTTOM_MIDDLE_CONE_Y = 2.1844;

    const double BLUE_SCORING_X = 1.8+0.068 + (0.0254 * 2);
    const double RED_SCORING_X = 14.74-0.068 - (0.0254 * 2);
    const double BLUE_PS_X = TAG_XY[3][0] - 0.7;
    const double RED_PS_X = TAG_XY[4][0] + 0.7;

    const double AUTO_DOCK_Y = 2.748;
    const double BLUE_AUTO_DOCK_X = 3.825875; // 2.412
    const double RED_AUTO_DOCK_X = 12.09675;  // 14.130

    const double BOTTOM_PIECE_Y = 0.919;
    const double BOTTOM_MID_PIECE_Y = 2.138;
    const double TOP_MID_PIECE_Y = 3.358;
    const double TOP_PIECE_Y = 4.577;

    const double BLUE_PIECE_X = 7.068;
    const double RED_PIECE_X = 9.474;
}

namespace InputConstants
{
    const int LJOY_PORT = 0;
    const int LJOY_X = 0;
    const int LJOY_Y = 1;

    const int RJOY_PORT = 1;
    const int RJOY_X = 0;
    const int RJOY_Y = 1;

    const int XBOX_PORT = 2;
    const int XBOX_LJOY_X = 0;
    const int XBOX_LJOY_Y = 1;
    const int XBOX_LTRIGGER = 2;
    const int XBOX_RTRIGGER = 3;
    const int XBOX_RJOY_X = 4;
    const int XBOX_RJOY_Y = 5;

    const int OUTAKE_BUTTON = 4; //6 for left joystick backup
    const int INTAKE_BUTTON = 3; //5 for right joystick backup
    const int LOCK_WHEELS_LOWER_BUTTON = 2; //4 for left joystick
    const int INCHING_LOWER_BUTTON = 2; //3 for right joystick
    const int AUTO_BALANCE_BUTTON = 4; //6 for right joystick

    const int A_BUTTON = 1;
    const int B_BUTTON = 2;
    const int X_BUTTON = 3;
    const int Y_BUTTON = 4;
    const int L_BUMPER = 5;
    const int R_BUMPER = 6;
    const int CONE_INTAKE_BUTTON = 7;
    const int FIELD_ORIENT_BUTTON = 8;

    const int BUTTON_BOARD_PORT = 3;
    const int B1 = 1; //1
    const int B2 = 2; //4
    const int B3 = 3; //7
    const int B4 = 4; //2
    const int B5 = 5; //5
    const int B6 = 6; //8
    const int B7 = 7; //3
    const int B8 = 8; //6
    const int B9 = 9; //9
    const int L3 = 12; //10
    const int L2 = 11; //11
    const int L1 = 10; //12
    const int BB_RIGHT = 15; //13
    //const int BB_DOWN = ; //14
    const int BB_UP = 13; //15
    const int BB_LEFT = 14; //16
    const int BB_X_TRIM_UP = 18;
    const int BB_X_TRIM_DOWN = 17;
    const int BB_Y_TRIM_UP = 16;
    const int BB_Y_TRIM_DOWN = 19;

}

namespace SwerveConstants
{
    const double WIDTH = 0.6858 + 0.152;
    const double LENGTH = 0.6858 + 0.152;
    const double WHEEL_DIAGONAL = 0.8128;
    const double TREAD_RADIUS = 0.0508;
    const double DRIVE_GEAR_RATIO = 1 / 6.12;
    const double SWIVEL_GEAR_RATIO = 1 / 12.8;
    const double MAX_TELE_VEL = 5.672;


    const double POSE_HISTORY_LENGTH = 0.3;
    const double CAMERA_DELAY = 0.1;

    const double INCHING_DIST = 0.0254;

    // const double trPosAngle = atan2((SwerveConstants::WIDTH/2), (SwerveConstants::LENGTH/2));
    // const double tlPosAngle = -trPosAngle;
    // const double brPosAngle = 180 - trPosAngle;
    // const double blPosAngle = trPosAngle - 180;

    const int TR_DRIVE_ID = 4;  // 13, 3
    const int TL_DRIVE_ID = 23; // 11, 1
    const int BR_DRIVE_ID = 1;  // 18, 4
    const int BL_DRIVE_ID = 22; // 15, 22

    const int TR_TURN_ID = 10; // 14, 5
    const int TL_TURN_ID = 5;  // 12, 7
    const int BR_TURN_ID = 7;  // 17, 10
    const int BL_TURN_ID = 19; // 16, 19

    const int TR_CANCODER_ID = 8; // 62, 2
    const int TL_CANCODER_ID = 2; // 10, 9
    const int BR_CANCODER_ID = 9; // 8, 8
    const int BL_CANCODER_ID = 6; // 42, 6

    const double TR_CANCODER_OFFSET = 173.75; // 172.7
    const double TL_CANCODER_OFFSET = 175.8;  // 177.02
    const double BR_CANCODER_OFFSET = 74.26;  // 75.85
    const double BL_CANCODER_OFFSET = 41.6;   // 43.2

    const double MAX_LA = 2.75;
    const double MAX_LV = 4;   //(past year: 3, 4) (stable but a bit too slow: 2, 4) (2.75, 4 for three piece)
    const double MAX_AA = 360; // 270
    const double MAX_AV = 540; // 450

    const double klV = 0.502636; // If you increase pd, check auto lineup
    const double klVI = -0.359672;
    const double klA = 4.11;
    const double klP = 2.5; //1.5
    const double klD = 0.05;

    const double kaV = 34.2064;
    const double kaVI = -25.4095;
    const double kaA = 0;
    const double kaP = 4; // 0.008
    const double kaD = 0;

    const double CLAW_MID_OFFSET = 0.0254 * 2;

    const double AUTOKTILT = 0.01;
    const double AUTODEADANGLE = 7.0;
    const double PITCHOFFSET = 2.0;
    const double ROLLOFFSET = 180.0 + 1;
}

namespace TwoJointArmConstants
{
    const double UPPER_ARM_LENGTH = 0.7493; // 0.7493 for 29.5
    const double FOREARM_LENGTH = 0.7239;   // 0.7239 for 28.5
    const double EE_LENGTH = 0.3429;        // 0.3429 for 13.5
    const double MOUNTING_HEIGHT = 0.4318; //0.4318 for 17
    const double CUBE_INTAKE_PIVOT_TO_SHOULDER_HEGHT = 0.3622548;
    const double CUBE_INTAKE_LENGTH = 0.403195028;
    const double CUBE_INTAKE_TO_SHOULDER_X = 0.29856176;
    const double CUBE_INTAKE_COLLISION_BUFFER = 0.3532;
    const double CONE_INTAKE_PIVOT_TO_SHOULDER_HEGHT = 0.3622548; // TODO get different stuffs with cone intake
    const double CONE_INTAKE_LENGTH = 0.403195028;
    const double CONE_INTAKE_TO_SHOULDER_X = 0.29856176;
    const double CONE_INTAKE_COLLISION_BUFFER = 0.3532; // 0.2032

    const double COLLISION_LOOKAHEAD_TIME = 0.65;

    const double SHOULDER_MIN_ANG = -90;
    const double SHOULDER_MAX_ANG = 90;
    const double ELBOW_MIN_ANG = 0;
    const double ELBOW_MAX_ANG = 360;

    const double SHOULDER_ARM_MAX_VEL = 135; //100, 270, 270, 200 was too fast
    const double ELBOW_ARM_MAX_VEL = 135; //90, 135, 90, 90 was too slow
    const double SHOULDER_ARM_MAX_ACC = 180;
    const double ELBOW_ARM_MAX_ACC = 180;

    const int SHOULDER_MASTER_ID = 6;
    const int SHOULDER_SLAVE_ID = 11;
    const int ELBOW_MASTER_ID = 8;
    const int ELBOW_SLAVE_ID = 3;
    const int SHOULDER_BRAKE_ID = 5;
    const int ELBOW_BRAKE_ID = 4;
    const int SHOULDER_ENCODER_ID = 0;
    const double SHOULDER_ENCODER_OFFSET = -67.4;

    const double UPPER_ARM_I = 0.206;
    const double FOREARM_I = 0.22; // 0.32, 0.35?

    const double UPPER_ARM_M = 1.937; // 2.53, 0.114
    const double FOREARM_M = 3.196;   // 1.41, 0.114

    const double UPPER_ARM_COM_DIST = 0.29; // 0.304, 0.381
    const double FOREARM_COM_DIST = 0.63;   // 0.26, 0.381

    const double SHOULDER_I = UPPER_ARM_I + UPPER_ARM_M * UPPER_ARM_COM_DIST * UPPER_ARM_COM_DIST;
    const double ELBOW_I = FOREARM_I + FOREARM_M * FOREARM_COM_DIST * FOREARM_COM_DIST;

    const double SHOULDER_KV = 13.7273; 
    const double SHOULDER_KVI = -8.95455; 
    const double ELBOW_KV = 45.3989;
    const double ELBOW_KVI = -25.9533;

    const double skD_ = 0.05; //0.05, 0.05, 0, 0.15
    const double skP_ = 0.1; //Unstable but works 0, 0.5, 0, 0.5
    const double ekD_ = 0;
    const double ekP_ = 0.15;

    const double SHOULDER_TO_ELBOW_RATIO = (30.0 / 48.0);   // 30:54 now 30:48
    const double MOTOR_TO_SHOULDER_RATIO = 1.0 / 243.911; // 1:194.4 (243.911)
    const double MOTOR_TO_ELBOW_RATIO = 1.0 / 43.556;     // 1:40.5 (43.556) for motor to shoulder area, 1:72.9 for motor to elbow joint with 54 tooth

    const double HIGH_CUBE_OUTAKE_VOLTS = -1;
    const double MID_CUBE_OUTAKE_VOLTS = -1;

    // const double ARM_POSITIONS[11][4] =
    //     {
    //         {0.0, -0.3175, 0, 179.999999}, // stowed, 0, 180
    //         {0.63102, -0.26025, 16.2, 140.5},    // cube intake, 16.2, 140.5
    //         {1.1607, 0.59354, 8, 90},     // mid, 8, 90 {1.15488, 0.55676, 8, 92}
    //         {-0.63102, -0.26025, -16.2, 219.5}, // cube intake other side, or just special paths!
    //         {1.54007, 0.92549, 49, 17},     // high, 49, 17
    //         {1.02963, 0.36889, 2.5, 108.35},     // cube mid, 2.5, 108.35
    //         {1.38, 0.7587, 24.96, 60.78},      // cube high, 24.96, 60.78
    //         {1.10306, 0.04506, 20.5, 107.5},        // ground but high, 20.5, 107.5
    //         {0.76201, 0.55449, -23.3, 120.5}, //ramming player station, -23.3, 120.5
    //         // {0.35105, -0.17901, -18.5, 165}, //Auto stow without cone intake, -18.5, 165
    //         {0.10074, -0.30109, -18.5, 179.99999}, //Auto stow with cone intake, -18.5, 180
    //         // {0.72371, -0.49738, 43, 125.5}, //Cone intake from the ground, 43, 125.5
    //         {0, 0, 25.7, 138} //Cone intake from intake, no trajectories, 25.7, 138
    // };

    const double ARM_POSITIONS[11][4] =
        {
            {0.0, -0.3175, 0, 179.999999}, // stowed, 0, 180
            {0.63102, -0.26025, 16.2, 140.5},    // cube intake, 16.2, 140.5
            {1.1607, 0.59354, 8, 90},     // mid, 8, 90 {1.15488, 0.55676, 8, 92}
            {-0.63102, -0.26025, -16.2, 219.5}, // cube intake other side, or just special paths!
            {1.54007, 0.92549, 49, 17},     // high, 49, 17
            {1.02963, 0.36889, 2.5, 108.35},     // cube mid, 2.5, 108.35
            {1.38, 0.7587, 24.96, 60.78},      // cube high, 24.96, 60.78

            // {1.10306, 0.04506, 20.5, 107.5},        // ground but high, 20.5, 107.5
            {0.72371, -0.49738, 43, 125.5}, //Cone intake from the ground, 43, 125.5

            {0.76201, 0.55449, -23.3, 120.5}, //ramming player station, -23.3, 120.5

            // {0.35105, -0.17901, -18.5, 165}, //Auto stow without cone intake, -18.5, 165
            {0.10074, -0.30109, -18.5, 179.99999}, //Auto stow with cone intake, -18.5, 180

            {0, 0, 25.7, 138} //Cone intake from intake, no trajectories, 25.7, 138
    };

    const int STOWED_NUM = 0;
    const int CUBE_INTAKE_NUM = 1;
    const int MID_NUM = 2;
    const int SPECIAL_NUM = 3;
    const int HIGH_NUM = 4;
    const int CUBE_MID_NUM = 5;
    const int CUBE_HIGH_NUM = 6;
    const int GROUND_NUM = 7;
    const int RAMMING_PLAYER_STATION_NUM = 8;
    const int AUTO_STOW_NUM = 9;
    const int CONE_INTAKE_NUM = 10;

    const double ANGLE_ERROR_THRESHOLD = 3;
    const double ANGLE_POS_KNOWN_THRESHOLD = 10;

    const double STALL_SAFETY = 250;

    const double SWINGTHROUGH_CLEARANCE = 15;

}

namespace ClawConstants
{
    const int PNEUMATIC_ID = 6;
    const int WHEEL_MOTOR_ID = 1;
    const double INTAKING_SPEED = 7;
    const double OUTAKING_SPEED = -3;
    const double RETAINING_SPEED = 0.5;
    const double RETAINING_CURRENT = 7;
}

namespace MotorIntakeConstants
{
    const int DEPLOYER_MOTOR_ID = 0; // TODO get value
    const int ROLLER_MOTOR_ID = 0;   // TODO get value

    const double ENCODER_DEPLOYED_TARGET = 512;

    const double DEPLOYER_MAX_VOLTAGE = 0; // TODO measure capped voltage for deployer
    const double ROLLER_MAX_VOLTAGE = 0;   // TODO measure voltage for cube/cone to actually pass through

    const double kP = 0; // TODO tune
    const double kI = 0; // TODO tune
    const double kD = 0; // TODO tune

    const double MAX_VELOCITY = M_PI / 2; // TODO tune - radians per second
    const double MAX_ACCELERATION = M_PI; // TODO tune - radians per second squared

    const double POS_ERR_TOLERANCE = 0.01; // TODO tune - error tolerance, in radians
    const double VEL_ERR_TOLERANCE = 0.1;  // TODO tune - error tolerance, in radians/s
} // namespace MotorIntakeConstants

namespace PneumaticsIntakeConstants
{
    const bool USING_CTRE = true; // if this is set to false, then you are using the REV pneumatics hub

    const int LEFT_SOLENOID_ID = 0;
    const int RIGHT_SOLENOID_ID = 7;
    const int ROLLER_MOTOR_ID = 13;

    const double ROLLER_INTAKE_VOLTAGE = -2.5;
    const double ROLLER_OUTTAKE_VOLTAGE = 2.5;
} // namespace PneumaticsIntakeConstants

namespace CubeGrabberConstants {
    const int LEFT_MOTOR_ID = 41;
    const int RIGHT_MOTOR_ID = 43;

    const double CLOCKWISE_VOLTAGE = 3;
    const double COUNTERCLOCKWISE_VOLTAGE = -3; 
}