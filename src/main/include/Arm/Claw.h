#pragma once

#include <iostream>
#include <math.h>
#include <string.h>

#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>

#include "Helpers/TrajectoryCalc.h"
#include "ArmConstants.h"

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