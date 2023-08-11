#pragma once

#include <vector>

#include "ControllerConstants.h"

namespace Actions{
    enum Action{
        NONE = -1,
        OUTAKE,
        INTAKE,
        AUTO_BALANCE,
        CONE_INTAKE,
        FIELD_ORIENT,
        ZERO_ARMS_1, // All 3 need to be pressed to zero
        ZERO_ARMS_2,
        ZERO_ARMS_3,
        MANUAL_CONTROL, //Enables manual control
        MANUAL_THETA,
        MANUAL_PHI,
        MANUAL_CONTROL_2, //Only controls theta arm
        MANUAL_THETA_2,
        SCORE,
        RAM_PLAYER_STATION,
        GO_MID,
        GO_HIGH,
        STOW,
        FLIP_ARM,
        FLIP_ARM_2,
        STOP_EVERYTHING,
        TOGGLE_CLAW,
        XSTRAFE,
        YSTRAFE,
        ROTATION,
        LOCK_WHEELS,
        SLOW_MODE, //Moves robot slowly to approach 
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };

    //Different enum for POV actions because logic is different
    enum POVAction{
        NO_POV_ACTION = -1,
        CUTOUT_INTAKE,
        CUTOUT_OUTAKE,
        STOW_POV,
        STOW_POV_2,
        INCH_UP,
        INCH_DOWN,
        INCH_LEFT,
        INCH_RIGHT,
        ACTION_COUNT_POV //Just the number of actions, as it is at the end of a enum
    };
}

namespace ControllerMapData{
    using namespace ControllerConstants;
    using namespace Actions;

    //Element in the buttonmap, mapping a single button to action
    struct ControlMapElement{
        Button button;
        Action action;
    };

    //Maps Buttons -> Actions
    //Buttons are structs in the form of {Joystick, ButtonData}
    //There are already some named ButtonData and Buttons
    const std::vector<ControlMapElement> ButtonMap = {
        {{LJOY, X_AXIS},     XSTRAFE},
        {{LJOY, Y_AXIS},        NONE},
        {{LJOY, TRIGGER},      SCORE},
        {{LJOY, B_2},    LOCK_WHEELS},
        {{LJOY, B_4},         OUTAKE},

        {{RJOY, X_AXIS},    ROTATION},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},TOGGLE_CLAW},
        {{RJOY, B_2},      SLOW_MODE},
        {{RJOY, B_3},         INTAKE},
        {{RJOY, B_4},   AUTO_BALANCE},
        
        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,    MANUAL_THETA}, 
        {XBOX_LJOY_Y,  MANUAL_THETA_2},
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,     MANUAL_PHI},
        {XBOX_LTRIGGER,  ZERO_ARMS_2},
        {XBOX_LTRIGGER,  MANUAL_CONTROL},
        {XBOX_RTRIGGER,  ZERO_ARMS_3},
        {XBOX_A_BUTTON ,        STOW},
        {XBOX_B_BUTTON ,      GO_MID},
        {XBOX_X_BUTTON ,  RAM_PLAYER_STATION},
        {XBOX_Y_BUTTON ,     GO_HIGH},
        {XBOX_L_BUMPER ,  STOP_EVERYTHING},
        {XBOX_R_BUMPER ,    FLIP_ARM},
        {{XBOX, B_7},    CONE_INTAKE},
        {{XBOX, B_8},   FIELD_ORIENT},

        {BB_UP,     MANUAL_CONTROL_2},
        {BB_LEFT,         FLIP_ARM_2},
        {BB_RIGHT,       ZERO_ARMS_1}
    };

    //Allows for maps of buttons to values, such as the index of the buttonboard
    //Only for buttons and triggers currently
    //No need for default val because it's now in the controller method
    template <typename T>
    struct ValueMapElement{
        Button button;
        T value;
    };

    const std::vector<ValueMapElement<int>> SCORING_POS = {
        {{BUTTONBOARD, B_1}, 1},
        {{BUTTONBOARD, B_2}, 2},
        {{BUTTONBOARD, B_3}, 3},
        {{BUTTONBOARD, B_4}, 4},
        {{BUTTONBOARD, B_5}, 5},
        {{BUTTONBOARD, B_6}, 6},
        {{BUTTONBOARD, B_7}, 7},
        {{BUTTONBOARD, B_8}, 8},
        {{BUTTONBOARD, B_9}, 9},
    };

    const std::vector<ValueMapElement<int>> GET_LEVEL = {
        {BB_L1, 1},
        {BB_L2, 2},
        {BB_L3, 3}
    };

    const std::vector<ValueMapElement<double>> GET_TRIM_X = {
        {BB_X_TRIM_UP, 1.0},
        {BB_X_TRIM_DOWN, -1.0}
    };

    const std::vector<ValueMapElement<double>> GET_TRIM_Y = {
        {BB_Y_TRIM_UP, 1.0},
        {BB_Y_TRIM_DOWN, -1.0}
    };

    //Takes the range from min to max
    //if range covers over 0, like from 350->10, the larger number comes first
    struct POVRange{
        int min;
        int max;
    };

    const POVRange POV_UP = {350, 10};
    const POVRange POV_RIGHT = {80, 100};
    const POVRange POV_DOWN = {170, 190};
    const POVRange POV_LEFT = {260, 280};

    struct POVMapElement{
        Button pov;
        POVRange range;
        POVAction action;
    };

    const std::vector<POVMapElement> POVMap = {
        {{RJOY, POV}, POV_UP, INCH_UP},
        {{RJOY, POV}, POV_DOWN, INCH_DOWN},
        {{RJOY, POV}, POV_LEFT, INCH_LEFT},
        {{RJOY, POV}, POV_RIGHT, INCH_RIGHT},

        {XBOX_POV, POV_UP, CUTOUT_INTAKE},
        {XBOX_POV, POV_DOWN, CUTOUT_OUTAKE},
        {XBOX_POV, POV_RIGHT, STOW_POV},
        {XBOX_POV, POV_LEFT, STOW_POV_2}
    };
};