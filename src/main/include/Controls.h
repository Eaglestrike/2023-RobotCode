#pragma once

#include <frc/Joystick.h>
#include "Constants.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

class Controls
{
    public:
        Controls();
        void periodic();

        double getXStrafe();
        double getYStrafe();
        double getTurn();
        bool lJoyTriggerPressed();
        bool rJoyTriggerPressed();
        bool outakePressed();
        bool intakePressed();

        double xboxLJoyX();
        double xboxLJoyY();
        double xboxRJoyX();
        double xboxRJoyY();

        bool aPressed();
        bool bPressed();
        bool xPressed();
        bool yPressed();
        bool rBumperPressed();
        bool lBumperPressed();
        bool rXTriggerPressed();
        bool lXTriggerPressed();
        bool dPadUpPressed();
        bool dPadDownPressed();
        bool dPadLeftPressed();
        bool dPadRightPressed();

        bool fieldOrient();

        int checkScoringButtons();

    private:
        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
        frc::Joystick buttonBoard_;
};