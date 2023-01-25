#include "Controls.h"

Controls::Controls() : xbox_{InputConstants::XBOX_PORT}
{
}

double Controls::getXStrafe()
{
    {
        return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    }
}

double Controls::getYStrafe()
{
    return -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
}