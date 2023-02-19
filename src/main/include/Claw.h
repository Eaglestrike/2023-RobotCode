#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include "TrajectoryCalc.h"

class Claw
{
    private:
        bool open_;
        double wheelSpeed_;

        frc::Solenoid clawPneumatic_;
        rev::CANSparkMax wheelMotor_;

    public:
        Claw();
        void periodic();

        bool isOpen();
        double wheelSpeed();

        void setOpen(bool open);
        void setWheelSpeed(double speed);
};