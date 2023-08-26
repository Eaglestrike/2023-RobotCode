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
        double getWithDeadband(Actions::Action action, double deadbandVal = ControllerConstants::DEFAULT_DEAD);
        double getWithDeadContinuous(Actions::Action action, double deadbandVal = ControllerConstants::DEFAULT_DEAD);
        bool getPressed(Actions::Action action);
        bool getPressedOnce(Actions::Action action);
        bool getTriggerDown(Actions::Action action, double defaultDown = ControllerConstants::DEFAULT_TRIGGER_DOWN);

        int getPOV(Actions::Action action);
        bool getPOVDown(Actions::POVAction action);
        bool getPOVDownOnce(Actions::POVAction action);

        /// @brief Returns value from a value map
        /// @tparam T should auto resolve, but know the type that it returns/uses
        /// @param map the mapping, founding in ControllerMapData
        /// @param defaultVal default value if none of the buttons are pressed
        /// @return the mapped button value, whichever one's first in the mapping
        template<typename T> T getValue(const std::vector<ControllerMapData::ValueMapElement<T>> (&map), T defaultVal){
            for(ControllerMapData::ValueMapElement<T> element : map){
                if(getButtonPressed(element.button)){
                    return element.value;
                }
            }
            return defaultVal;
        }

        /// @brief Returns value from a value map
        /// @tparam T should auto resolve, but know the type that it returns/uses
        /// @param map the mapping, founding in ControllerMapData
        /// @param defaultVal default value if none of the buttons are pressed
        /// @return the mapped button value, whichever one's first in the mapping
        template<typename T> T getValueOnce(const std::vector<ControllerMapData::ValueMapElement<T>> (&map), T defaultVal){
            for(ControllerMapData::ValueMapElement<T> element : map){
                if(getButtonPressedOnce(element.button)){
                    return element.value;
                }
            }
            return defaultVal;
        }

        //Buttons
        bool getButtonPressed(ControllerConstants::Button button);
        bool getButtonPressedOnce(ControllerConstants::Button button);

        //Calls all buttons and triggers to not buffer (disabledPeriodic)
        void stopBuffer();

    private:
        //Maps Actions -> Buttons
        ControllerConstants::Button actionMap_[ControllerMapData::ACTION_COUNT];
        bool wasPressed[ControllerMapData::ACTION_COUNT];

        //Maps PovActions -> POV range on a POV
        std::pair<ControllerConstants::Button, ControllerMapData::POVRange> actionMapPOV_[ControllerMapData::ACTION_COUNT_POV];
        bool wasPressedPOV[ControllerMapData::ACTION_COUNT_POV];

        //Array of all the Joysticks
        //Names of controllers are in ControllerConstants.h
        frc::Joystick* joysticks_[ControllerConstants::NUM_JOYSTICKS];
};