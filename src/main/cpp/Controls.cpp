#include "Controls.h"

Controls::Controls() : xbox_{InputConstants::XBOX_PORT}
{
}

double Controls::getXStrafe()
{
    double x = xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    if(abs(x) < 0.1){
        return 0.0;
    }
    return x;
}

double Controls::getYStrafe()
{
    double y = -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    if(abs(y) < 0.1){
        return 0.0;
    }
    return y;
}

bool Controls::A_IsPressed(){
    return xbox_.GetRawButtonPressed(1);
}