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

using namespace std;

#define M_PI 3.14159265358979323846 //for simulation

namespace GeneralConstants
{
    const double Kdt = 0.005; //0.005, 0.02
    const int MAX_RPM = 6380;
    const int TICKS_PER_ROTATION = 2048;

    const double FREE_SPEED = 6380;
    const double FREE_CURRENT = 1.5;
    const double STALL_CURRENT = 257;
    const double MAX_VOLTAGE = 12;

    const double RESISTANCE = MAX_VOLTAGE/STALL_CURRENT;
    const double Kv = ((FREE_SPEED * 2 * M_PI) / 60 ) / (MAX_VOLTAGE - FREE_CURRENT * RESISTANCE);
}

//Note: still contains constants from 2022 season
namespace LimelightConstants
{
    const double CAMERA_PITCH = 34; //49.5 from 0 at top (90-49.5), 30
    const double CAMERA_HEIGHT = 0.9652; //0.533, 0.7874, 0.9144 raised
    const double TURRET_ANGLE_OFFSET = 4.75; //5.0, 4?
    const double TURRET_CENTER_RADIUS = 0.127;
    const double ROBOT_TURRET_CENTER_DISTANCE = 0.127;
    const double LIMELIGHT_TO_BALL_CENTER_DIST = -0.2032; //TODO get more precise, -0.1145, -0.2032?

    const double TARGET_HEIGHT_UPPER = 2.641;
    const double TARGET_HEIGHT_LOWER = TARGET_HEIGHT_UPPER - 0.0508;
    const double GOAL_RADIUS = 0.6096;
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
}

namespace SwerveConstants
{
    const units::meter_t HALF_WIDTH = 0.3683_m;
    const double TICKS_PER_REV = 12650;

    //TODO: tune
    const units::meters_per_second_t MAX_SPEED = 4_mps;
    const units::radians_per_second_t MAX_ROT = units::radians_per_second_t{4*M_PI}; //2 revolutions per second

    const string testPath = "3mTest.wpilib.json";

    //can tune, using safe defaults
    const units::meters_per_second_t kMaxSpeed = 1_mps;
    const units::meters_per_second_squared_t kMaxAccel = 0.5_m / (1_s * 1_s);

    const int FRspeedPort = 13; //13, 3
    const int FLspeedPort = 11; //11, 1
    const int BRspeedPort = 18; //18, 4
    const int BLspeedPort = 22; //15, 22

    const int FRanglePort = 14; //14, 5
    const int FLanglePort = 12; //12, 7
    const int BRanglePort = 17; //17, 10
    const int BLanglePort = 19; //16, 19

    const int FRencoder = 62; //62, 2
    const int FLencoder = 10; //10, 9
    const int BRencoder = 8; //8, 8
    const int BLencoder = 6; //42, 6
    
    const double FROFF = 19.77; //19.77, 
    const double FLOFF = 109.952; //109.952
    const double BROFF = 197.5; //197.5
    const double BLOFF = -139.92 + 180; // 356.39

    const auto ks = 0.24387_V;
    const auto kv = 2.1675 * 1_V * 1_s / 1_m;
    const auto ka = 0.27807 * 1_V * 1_s * 1_s / 1_m;
    
    //WPILib says these are good?
    const units::unit_t<frc::RamseteController::b_unit> kRamseteB = units::unit_t<frc::RamseteController::b_unit>{2};
    const units::unit_t<frc::RamseteController::zeta_unit> kRamseteZeta = units::unit_t<frc::RamseteController::zeta_unit>{0.7};

    //turning swerve motor pid
    const double P = 0.06;
    const double I = 0.1;
    const double D = 0.0;

    //driving swerve motor pid
    const double sP = 0.0; //6; 
    const double sI = 0.0;
    const double sD = 0.0;

}
