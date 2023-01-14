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
    const int TICKS_PER_ROTATION = 2048;
    const double TICKS_PER_RADIAN = TICKS_PER_ROTATION/(2*M_PI);
}

namespace ArmConstants{
    const int BASE_MOTOR_ID = 0;
    const int TOP_MOTOR_ID = 0;

    const double BASE_ARM_LENGTH = 0.0;
    const double TOP_ARM_LENGTH = 0.0;
    const double PIVOT_HEIGHT = 0.0;

    const double BASE_PID[3] = {0.0, 0.0, 0.0};
    const double TOP_PID[3] = {0.0, 0.0, 0.0};
}
