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

        frc::Solenoid clawPneumatic_;
        rev::CANSparkMax wheelMotor_;

    public:
        enum WheelState
        {
            INTAKING,
            OUTAKING,
            IDLE
        };
        WheelState wheelState_;
        Claw();
        void periodic();

        bool isOpen();
        WheelState getWheelState();

        void setOpen(bool open);
        void setWheelState(WheelState wheelState);
};