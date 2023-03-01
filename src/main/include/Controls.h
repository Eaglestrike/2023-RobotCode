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
        bool lJoyTriggerDown();
        bool rJoyTriggerPressed();
        bool outakePressed();
        bool intakePressed();
        bool lLowerButtonPressed();
        bool rLowerButtonPressed();
        bool autoBalanceDown();

        double xboxLJoyX();
        double xboxLJoyY();
        double xboxRJoyX();
        double xboxRJoyY();

        bool aDown();
        bool bDown();
        bool xDown();
        bool yDown();
        bool rBumperPressed();
        bool lBumperDown();
        bool rXTriggerDown();
        bool lXTriggerDown();
        bool dPadUpDown();
        bool dPadDownDown();
        bool dPadLeftPressed();
        bool dPadRightPressed();

        bool fieldOrient();

        int checkScoringButtons();

    private:
        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
        frc::Joystick buttonBoard_;

        bool dPadLeftDown_, dPadRightDown_, intakeDown_, outakeDown_;
};