#pragma once

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

    const double TRIMMING_DIST = 0.0254;
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

    const double TR_CANCODER_OFFSET = -12.5 + 180; // 172.7
    const double TL_CANCODER_OFFSET = -7.29 + 180;  // 177.02
    const double BR_CANCODER_OFFSET = -31.3 + 180;  // 75.85
    const double BL_CANCODER_OFFSET = -96.3 + 180;   // 43.2

    const double MAX_LA = 2.75;// Max Linear Acceleration
    const double MAX_LV = 4;   // Max Linear Velocity (past year: 3, 4) (stable but a bit too slow: 2, 4) (2.75, 4 for three piece)
    const double MAX_AA = 360; // 270
    const double MAX_AV = 540; // 450

    const double klV = 0.502636; // If you increase pd, check auto lineup
    const double klVI = -0.359672; //Vel offset
    const double klA = 4.11;
    const double klP = 2.5; //linear kp :1.5
    const double klD = 0.05;

    const double kaV = 34.2064;
    const double kaVI = -25.4095;
    const double kaA = 0;
    const double kaP = 4; // 0.008
    const double kaD = 0;

    namespace ModuleConstants{
        const double maxV = 1440;
        const double maxA = 14400 * 10;
        const double kP = 0.05;
        const double kD = 0;
        const double kV = 1 / 261.864;
        const double kVI = -131.727;
        const double kA = 0;

        const double akP_ = 0.1; //COULDO tune values 0.08, 0, 0.001 (0.1, 0, 0.001)
        const double akI_ = 0.0;
        const double akD_ = 0.001;

        const double dkP_ = 0.0;
        const double dkI_ = 0.0;
        const double dkD_ = 0.0;
    };

    const double CLAW_MID_OFFSET = 0.0254 * 4.5; //was 2

    const double AUTOKTILT = 0.01;
    const double AUTODEADANGLE = 10.0;
    const double PITCHOFFSET = 0.0;
    const double ROLLOFFSET = 180.0 + 2;
    const double MIN_TILT_ON_STATION = 12;

    const double PRE_SENDING_IT_SPEED = 0.25;
    const double SENDING_IT_FAST_SPEED = 0.4;
    const double SENDING_IT_MED_SPEED = 0.35; //Was 0.3

    const double SENDING_IT_TIME = 1.1; //was 0.9
}