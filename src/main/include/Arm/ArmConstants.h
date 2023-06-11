#pragma once

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
    const double MAX_X_EXENTIONS = 1.6021;

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
    const double SHOULDER_ENCODER_OFFSET = -67.4 - 60;

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
    const double skP_ = 0.2; //Unstable but works 0, 0.5, 0, 0.5
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

            // {0.63102, -0.26025, 16.2, 140.5},    // cube intake, 16.2, 140.5
            {0.68919, -0.19095, 13, 138},       // cube intake, 13, 138

            {1.1607, 0.59354, 8, 90},     // mid, 8, 90 {1.15488, 0.55676, 8, 92}

            // {-0.63102, -0.26025, -16.2, 219.5}, // cube intake other side, or just special paths!
            {-0.68919, -0.19095, -13, 222},       // cube intake other side, or just special paths!

            {1.54007, 0.92549, 49, 17},     // high, 49, 17
            {1.02963, 0.36889, 2.5, 108.35},     // cube mid, 2.5, 108.35
            {1.38, 0.7587, 24.96, 60.78},      // cube high, 24.96, 60.78

            // {1.10306, 0.04506, 20.5, 107.5},        // ground but high, 20.5, 107.5
            {0.72371, -0.49738, 43, 125.5}, //Cone intake from the ground, 43, 125.5

            // {0.76201, 0.55449, -23.3, 120.5}, //ramming player station, -23.3, 120.5
            // {0.6685, 0.53897, -31.7, 127}, //ramming player station closer to bumper, -31.7, 127
            {0.70918, 0.55008, -28, 124}, //ramming player station, far but not as far, -28, -124

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
    const double INTAKING_SPEED = 10;
    const double OUTAKING_SPEED = -3;
    const double RETAINING_SPEED = 0.5;
    const double RETAINING_CURRENT = 7;
}