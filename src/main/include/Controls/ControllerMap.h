#pragma once

#include <vector>

#include "ControllerConstants.h"

namespace Actions{
    enum Action{
        NONE = -1,
        OUTAKE,
        INTAKE,
        LOCK_WHEELS_LOWER,
        INCHING_LOWER,
        AUTO_BALANCE,
        CONE_INTAKE,
        FIELD_ORIENT,
        ZERO_ARMS_1, // All 3 need to be pressed to zero
        ZERO_ARMS_2,
        ZERO_ARMS_3,
        ACTION_COUNT //Just the number of actions, as it is at the end of a enum
    };
}

namespace ControllerMapData{
    using namespace ControllerConstants;
    using namespace Actions;

    struct ControlMapElement{
        Button button;
        Action action;
    };

    //Maps Buttons -> Actions
    //Buttons are structs in the form of {Joystick, ButtonData}
    //There are already some named ButtonData and Buttons
    const std::vector<ControlMapElement> ButtonMap = {
        {{LJOY, X_AXIS},        NONE},
        {{LJOY, Y_AXIS},        NONE},
        {{LJOY, TRIGGER},       NONE},

        {{RJOY, X_AXIS},        NONE},
        {{RJOY, Y_AXIS},        NONE},
        {{RJOY, TRIGGER},       NONE},
        {{RJOY, B_4},   AUTO_BALANCE},
        
        {XBOX_LJOY_X,           NONE},
        {XBOX_LJOY_Y,           NONE}, 
        {XBOX_LTRIGGER,  ZERO_ARMS_2},
        {XBOX_RTRIGGER,  ZERO_ARMS_3},
        {XBOX_RJOY_X,           NONE},
        {XBOX_RJOY_Y,           NONE},
        {XBOX_A_BUTTON ,        NONE},
        {XBOX_B_BUTTON ,        NONE},
        {XBOX_X_BUTTON ,        NONE},
        {XBOX_Y_BUTTON ,        NONE},
        {XBOX_L_BUMPER ,        NONE},
        {XBOX_R_BUMPER ,        NONE},
        {{XBOX, B_7},    CONE_INTAKE},
        {{XBOX, B_8},   FIELD_ORIENT},

        {BB_RIGHT,       ZERO_ARMS_1}
    };
};