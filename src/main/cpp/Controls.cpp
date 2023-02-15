#include "Controls.h"

Controls::Controls() : xbox_{InputConstants::XBOX_PORT}
{
}

double Controls::getXStrafe()
{
    double x = xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    if(abs(x) < 0.2){
        return 0.0;
    }
    return x;
}

double Controls::getYStrafe()
{
    double y = -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    if(abs(y) < 0.2){
        return 0.0;
    }
    return y;
}

double Controls::getX2Strafe()
{
    double x = xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X);
    if(abs(x) < 0.2){
        return 0.0;
    }
    return x;
}

double Controls::getY2Strafe()
{
    double y = -xbox_.GetRawAxis(InputConstants::XBOX_RJOY_Y);
    if(abs(y) < 0.2){
        return 0.0;
    }
    return y;
}

bool Controls::A_IsPressed(){
    return xbox_.GetRawButtonPressed(InputConstants::A_BUTTON);
}

bool Controls::B_IsPressed(){
    return xbox_.GetRawButtonPressed(InputConstants::B_BUTTON);
}

bool Controls::X_IsPressed(){
    return xbox_.GetRawButtonPressed(InputConstants::X_BUTTON);
}

bool Controls::Y_IsPressed(){
    return xbox_.GetRawButtonPressed(InputConstants::Y_BUTTON);
}