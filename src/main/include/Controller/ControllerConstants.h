#pragma once

#include <vector>
#include <utility>

namespace ControllerConstants{
    const double DEFAULT_DEAD = 0.05;
    const double DEFAULT_TRIGGER_DOWN = 0.75; //If the trigger is pressed >0.75, it is down

    enum Joysticks{
        NO_JOYSTICK = -1,
        LJOY,
        RJOY,
        XBOX,
        BUTTONBOARD,
        NUM_JOYSTICKS //Counts the number of Joysticks since this is an enum
    };

    typedef std::pair<Joysticks, int> JoystickPortMap;
    /// maps the joysticks to their ports
    const std::vector<JoystickPortMap> JOYSTICK_PORTS = {
        {LJOY, 0},
        {RJOY, 1},
        {XBOX, 2},
        {BUTTONBOARD, 3}
    };

    //Holds information of what to return: double or bool and what function to call
    enum ButtonType{
        NO_BUTTON_TYPE = -1,
        AXIS_BUTTON, //Axis, which returns a double not boolean
        BUTTON_BUTTON, //Normal "Push down to activate" button
        TRIGGER_BUTTON, //Trigger on a joystick (sometimes a "trigger" is an axis), has no button id
        POV_BUTTON //8 button button -> READ CODE/DOCUMENTATION BECAUSE IT'S FUNKY
    };

    //Data is the type of button and the id
    //ID for POV is still the id
    struct ButtonData{
        ButtonType type;
        int id;
    };

    //Button data for generic joystics
    const ButtonData NO_BUTTON = {ButtonType::NO_BUTTON_TYPE, -1};
    const ButtonData POV = {ButtonType::POV_BUTTON, -1}; //-1 because no button id
    const ButtonData TRIGGER = {ButtonType::TRIGGER_BUTTON, -1}; //-1 because no button id
    const ButtonData X_AXIS = {ButtonType::AXIS_BUTTON, 0};
    const ButtonData Y_AXIS = {ButtonType::AXIS_BUTTON, 1};
    const ButtonData B_1 = {ButtonType::BUTTON_BUTTON, 1};
    const ButtonData B_2 = {ButtonType::BUTTON_BUTTON, 2};
    const ButtonData B_3 = {ButtonType::BUTTON_BUTTON, 3};
    const ButtonData B_4 = {ButtonType::BUTTON_BUTTON, 4};
    const ButtonData B_5 = {ButtonType::BUTTON_BUTTON, 5};
    const ButtonData B_6 = {ButtonType::BUTTON_BUTTON, 6};
    const ButtonData B_7 = {ButtonType::BUTTON_BUTTON, 7};
    const ButtonData B_8 = {ButtonType::BUTTON_BUTTON, 8};
    const ButtonData B_9 = {ButtonType::BUTTON_BUTTON, 9};

    //Default is None
    struct Button{
        Joysticks joystick = NO_JOYSTICK;
        ButtonData data = {NO_BUTTON_TYPE, -1};
    };

    const Button XBOX_POV = {XBOX, POV};
    const Button XBOX_LJOY_X     = {XBOX, {AXIS_BUTTON,   0}};
    const Button XBOX_LJOY_Y     = {XBOX, {AXIS_BUTTON,   1}};
    const Button XBOX_LTRIGGER   = {XBOX, {AXIS_BUTTON,   2}};
    const Button XBOX_RTRIGGER   = {XBOX, {AXIS_BUTTON,   3}};
    const Button XBOX_RJOY_X     = {XBOX, {AXIS_BUTTON,   4}};
    const Button XBOX_RJOY_Y     = {XBOX, {AXIS_BUTTON,   5}};
    const Button XBOX_A_BUTTON   = {XBOX, {BUTTON_BUTTON, 1}};
    const Button XBOX_B_BUTTON   = {XBOX, {BUTTON_BUTTON, 2}};
    const Button XBOX_X_BUTTON   = {XBOX, {BUTTON_BUTTON, 3}};
    const Button XBOX_Y_BUTTON   = {XBOX, {BUTTON_BUTTON, 4}};
    const Button XBOX_L_BUMPER   = {XBOX, {BUTTON_BUTTON, 5}};
    const Button XBOX_R_BUMPER   = {XBOX, {BUTTON_BUTTON, 6}};

    //BB stands for ButtonBoard
    const Button BB_L3 = {BUTTONBOARD, {BUTTON_BUTTON, 12}}; //10
    const Button BB_L2 = {BUTTONBOARD, {BUTTON_BUTTON, 11}}; //11
    const Button BB_L1 = {BUTTONBOARD, {BUTTON_BUTTON, 10}}; //12
    const Button BB_UP          = {BUTTONBOARD, {BUTTON_BUTTON, 13}}; //15
    const Button BB_LEFT        = {BUTTONBOARD, {BUTTON_BUTTON, 14}}; //16
    const Button BB_RIGHT       = {BUTTONBOARD, {BUTTON_BUTTON, 15}}; //13
    const Button BB_X_TRIM_UP   = {BUTTONBOARD, {BUTTON_BUTTON, 18}};
    const Button BB_X_TRIM_DOWN = {BUTTONBOARD, {BUTTON_BUTTON, 17}};
    const Button BB_Y_TRIM_UP   = {BUTTONBOARD, {BUTTON_BUTTON, 16}};
    const Button BB_Y_TRIM_DOWN = {BUTTONBOARD, {BUTTON_BUTTON, 19}};
}