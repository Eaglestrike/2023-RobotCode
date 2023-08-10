#pragma once

#include <frc/Joystick.h>

#include "ControllerConstants.h"
#include "ControllerMap.h"

class Controller{
    public:
        union Output{
            double doubleVal;
            bool boolVal;
        };
        Controller();

        //Deadband Functions
        static        bool isDead(Output value,     double deadbandVal = ControllerConstants::DEFAULT_DEAD);
        static inline bool isDead(double value,     double deadbandVal = ControllerConstants::DEFAULT_DEAD);
        static        double Deadband(Output value, double deadbandVal = ControllerConstants::DEFAULT_DEAD);
        static inline double Deadband(double value, double deadbandVal = ControllerConstants::DEFAULT_DEAD);

        //Get Functions
        Output get(Actions::Action action);
        double getRawAxis(Actions::Action action);
        double getDead(Actions::Action action, double deadbandVal = 0.1);
        bool getPressed(Actions::Action action);
        bool getTriggerDown(Actions::Action action, double defaultDown = ControllerConstants::DEFAULT_TRIGGER_DOWN);

    private:
        //Maps Actions -> Buttons
        ControllerConstants::Button actionMap_[ControllerMapData::ACTION_COUNT];

        //Array of all the Joysticks
        //Names of controllers are in ControllerConstants.h
        frc::Joystick* joysticks_[ControllerConstants::NUM_JOYSTICKS];
};