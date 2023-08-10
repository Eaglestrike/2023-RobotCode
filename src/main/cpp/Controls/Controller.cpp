#include "Controller\Controller.h"

#include <iostream>

using namespace ControllerMapData;
using namespace ControllerConstants;

/**
 * Default Constructor for Controller
*/
Controller::Controller(){
    //Initialize Joysticks
    for(JoystickPortMap map: JOYSTICK_PORTS){
        int joystickID = map.first;
        if( (joystickID < 0) || (joystickID >= NUM_JOYSTICKS) ){ // If the joystick id is out of bounds
            std::cout<<"Bad joystick id "<<joystickID<<std::endl;
        }
        else{
            joysticks_[joystickID] = new frc::Joystick(map.second);
        }
    }
    //check if any joysticks are nullptr
    for(int i = 0; i<NUM_JOYSTICKS; i++){
        if(joysticks_[i]){
            std::cout<<"No joystick map for id " << i<< std::endl;
        }
    }

    //Initialize the mapping from Actions -> Buttons 
    for(ControlMapElement mapElement : ControllerMapData::ButtonMap){ //Loop through all pairings, assigning the action to the button
        if(mapElement.action == Action::NONE){
            continue;
        }
        else if(actionMap_[mapElement.action].joystick != Joysticks::NO_JOYSTICK){
            std::cout<<"Double Mapping for Action " << mapElement.action<<std::endl;
        }
        else{
            actionMap_[mapElement.action] = mapElement.button;
        }
    }
}

/**
 * Checks if the value is within the deadband range
 * 
 * Basically if the abs(value) < deadbandVal, value = 0.
 * 
 * Will make unexpected behavior if the value was written as a bool.
 * @param value the output value from the controller
 * @param deadbandVal value range
*/
bool Controller::isDead(Output value, double deadbandVal){
    return isDead(value.doubleVal, deadbandVal);
}

/**
 * Checks if the value is within the deadband range
 * 
 * Basically if the abs(value) < deadbandVal, value = 0.
 * 
 * @param value the output value from the controller
 * @param deadbandVal value range
*/
inline bool Controller::isDead(double value, double deadbandVal){
    return abs(value) < deadbandVal;
}

/**
 * Applies the deadband to an output.
 * 
 * Basically if the abs(value) < deadbandVal, value = 0.
 * Useful in the case of joystick drift.
 * 
 * Will make unexpected behavior if the value was written as a bool.
 * @param value the output value from the controller
 * @param deadbandVal value range in which the output will be reduced to 0
*/
double Controller::Deadband(Output value, double deadbandVal){
    return Deadband(value.doubleVal, deadbandVal);
}

/**
 * Applies the deadband to an output.
 * 
 * Basically if the abs(value) < deadbandVal, value = 0.
 * Useful in the case of joystick drift.
 * 
 * @param value the output value from the controller
 * @param deadbandVal value range in which the output will be reduced to 0
*/
inline double Controller::Deadband(double value, double deadbandVal){
    return (abs(value)<deadbandVal)? 0.0 : value;
}

/**
 * Gets the data from the controllers, based of an action key
 * 
 * double returns should always be in the [-1.0, 1.0] range
 * 
 * @param action the action key for the date, reference is in ControllerMap.h
 * @returns a union of double and bool, depending on whenever the button is an axis or button
*/
Controller::Output Controller::get(Action action){
    Output o;
    Button button = actionMap_[action];
    switch(button.data.type){
        case AXIS_BUTTON:
            o.doubleVal = joysticks_[button.joystick]->GetRawAxis(button.data.id);
            break;
        case BUTTON_BUTTON:
            o.boolVal = joysticks_[button.joystick]->GetRawButtonPressed(button.data.id);
            break;
        case TRIGGER_BUTTON:
            o.boolVal = joysticks_[button.joystick]->GetTriggerPressed();
            break;
        default:
            std::cout<<"Bad Button Mapping for Action"<< action << std::endl;
    };
    return o;
}

/**
 * Gets the data from an axis.
 * 
 * returns should always be in the [-1.0, 1.0] range.
 * 
 * prints error if bad action and returns 0
 * 
 * @param action the action key for the date, reference is in ControllerMap.h
 * @returns a double in the range [-1.0, 1.0] (not enforced)
*/
double Controller::getRawAxis(Action action){
    Button button = actionMap_[action];
    switch(button.data.type){
        case AXIS_BUTTON:
            return joysticks_[button.joystick]->GetRawAxis(button.data.id);
        case BUTTON_BUTTON:
        case TRIGGER_BUTTON:
            std::cout<<"Not applicable for Raw Axis: Action " << action << " call"<<std::endl;
            break;
        default:
            std::cout<<"Bad Button Mapping for Action" << action << std::endl;
    };
    return 0.0;
}

/**
 * Gets the data from an axis and applies a deadband
 * 
 * double returns should always be in the [-1.0, 1.0] range
 * 
 * prints error if bad action and returns 0
 * 
 * @param action the action key for the date, reference is in ControllerMap.h
 * @param deadbandVal value range in which the output will be reduced to 0
 * @returns a double in the range [-1.0, 1.0] (not enforced)
*/
double Controller::getDead(Action action, double deadbandVal){
    Button button = actionMap_[action];
    double raw;
    switch(button.data.type){
        case AXIS_BUTTON:
            raw = joysticks_[button.joystick]->GetRawAxis(button.data.id);
            return Deadband(raw, deadbandVal);
        case BUTTON_BUTTON:
        case TRIGGER_BUTTON:
            std::cout<<"Not applicable for Deadband: Action " << action << " call"<<std::endl;
            break;
        default:
            std::cout<<"Bad Button Mapping for Action" << action << std::endl;
    };
    return 0.0;
}

/**
 * Gets if the action's button is pressed
 * 
 * prints error if bad action and returns false
 * 
 * @param action the action key for the date, reference is in ControllerMap.h
 * @returns if the button is pressed
*/
bool Controller::getPressed(Action action){
    Button button = actionMap_[action];
    switch(button.data.type){
        case AXIS_BUTTON:
            std::cout<<"Not applicable for getPressed: Action " << action << " call"<<std::endl;
            break;
        case BUTTON_BUTTON:
            return joysticks_[button.joystick]->GetRawButtonPressed(button.data.id);
        case TRIGGER_BUTTON:
            return joysticks_[button.joystick]->GetTriggerPressed();
        default:
            std::cout<<"Bad Button Mapping for Action"<< action << std::endl;
    };
    return false;
}

/**
 * If the actions a trigger, it returns the get value
 * Gets if the action's axis's value is greater than the defaultDown
 * 
 * prints error if bad action and returns false
 * 
 * @param action the action key for the date, reference is in ControllerMap.h
 * @param defaultDown the min for a trigger to be "down", default is 0.75
 * @returns if the trigger is pressed
*/

bool Controller::getTriggerDown(Action action, double defaultDown){
    Button button = actionMap_[action];
    double value;
    switch(button.data.type){
        case AXIS_BUTTON:
            value = joysticks_[button.joystick]->GetRawAxis(button.data.id);
            return value > defaultDown;
        case BUTTON_BUTTON:
            std::cout<<"Not applicable for getTriggerDown: Action " << action << std::endl;
            break;
        case TRIGGER_BUTTON:
            return joysticks_[button.joystick]->GetTriggerPressed();
        default:
            std::cout<<"Bad Button Mapping for Action" << action << std::endl;
    };
    return flase;
}
