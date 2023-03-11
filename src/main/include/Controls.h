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
        bool rLowerButton();
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
        bool dPadUpPressed();
        bool dPadDownDown();
        bool dPadLeftPressed();
        bool dPadRightPressed();
        bool bbUpDown();

        bool inchingUpDown();
        bool inchingDownDown();
        bool inchingLeftDown();
        bool inchingRightDown();

        bool fieldOrient();

        int checkScoringButtons();
        int checkLevelButtons();
        int checkPSButtons();

    private:
        frc::Joystick lJoy_;
        frc::Joystick rJoy_;
        frc::Joystick xbox_;
        frc::Joystick buttonBoard_;

        bool dPadUpDown_, dPadLeftDown_, dPadRightDown_, /*intakeDown_, outakeDown_,*/ inchingUpDown_, inchingDownDown_, inchingLeftDown_, inchingRightDown_;
};