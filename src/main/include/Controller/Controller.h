#pragma once

#include <utility>

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
        //Actions
        Output get(Actions::Action action);
        double getRawAxis(Actions::Action action);
        double getDead(Actions::Action action, double deadbandVal = 0.1);
        bool getPressed(Actions::Action action);
        bool getTriggerDown(Actions::Action action, double defaultDown = ControllerConstants::DEFAULT_TRIGGER_DOWN);

        int getPOV(Actions::Action action);
        bool getPOVDown(Actions::POVAction action);

        //Buttons
        bool getButtonPressed(ControllerConstants::Button button);

        //Returns value from a value map
        template<typename T> T getValue(const ControllerMapData::ValueMapElement<T> map[]){
            T* defaultVal; //Pointer cuz T might need constructor
            for(ControllerMapData::ValueMapElement<T> element : map){
                if(element.button.data.type == ControllerConstants::NO_BUTTON_TYPE){ //Get default val
                    defaultVal = *element.value;
                    continue;
                }
                if(getPressed(element.button)){
                    return element.value;
                }
            }
            return &defaultVal;
        };

        //Calls all buttons and triggers to not buffer (disabledPeriodic)
        void stopBuffer();

    private:
        //Maps Actions -> Buttons
        ControllerConstants::Button actionMap_[ControllerMapData::ACTION_COUNT];
        //Maps PovActions -> POV range on a POV
        std::pair<ControllerConstants::Button, ControllerMapData::POVRange> actionMapPOV_[ControllerMapData::ACTION_COUNT_POV];

        //Array of all the Joysticks
        //Names of controllers are in ControllerConstants.h
        frc::Joystick* joysticks_[ControllerConstants::NUM_JOYSTICKS];
};