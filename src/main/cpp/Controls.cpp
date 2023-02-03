#include "Controls.h"

Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT}
{
    //idk
}

void Controls::periodic()
{

}

double Controls::getXStrafe()
{
    /*if(manuallyOverrideTurret()) //For child proofing
    {
        return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    }*/

    //double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
    double x = xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    if(abs(x) < 0.05)
    {
        return 0;
    }
    x = (x > 0) ? (x - 0.05) / 0.95 : (x + 0.05) / 0.95;
    return x;
}

double Controls::getYStrafe()
{
    /*if(manuallyOverrideTurret()) //For child proofing
    {
        return -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    }*/

    //double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    double y = -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    if(abs(y) < 0.05)
    {
        return 0;
    }
    y = (y > 0) ? (y - 0.05) / 0.95 : (y + 0.05) / 0.95;
    return y;
}

double Controls::getTurn()
{
    /*if(manuallyOverrideTurret())
    {
        return 0;
    }*/

    //double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    double turn = xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X);
    if(abs(turn) < 0.05)
    {
        return 0;
    }
    turn = (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;

    return turn;
}

bool Controls::lJoyTriggerPressed()
{
    return lJoy_.GetTrigger();
}

bool Controls::rJoyTriggerPressed()
{
    return rJoy_.GetTrigger();
}

bool Controls::aPressed()
{
    return xbox_.GetRawButton(InputConstants::A_BUTTON);
}

bool Controls::bPressed()
{
    return xbox_.GetRawButton(InputConstants::B_BUTTON);
}

bool Controls::xPressed()
{
    return xbox_.GetRawButton(InputConstants::X_BUTTON);
}

bool Controls::yPressed()
{
    return xbox_.GetRawButton(InputConstants::Y_BUTTON);
}

bool Controls::rBumperPressed()
{
    return xbox_.GetRawButton(InputConstants::R_BUMPER);
}

bool Controls::lBumperPressed()
{
    return xbox_.GetRawButton(InputConstants::L_BUMPER);
}

bool Controls::fieldOrient()
{
    return xbox_.GetRawButton(InputConstants::FIELD_ORIENT_BUTTON);
}

bool Controls::rXTriggerPressed()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_RTRIGGER) > 0.75;
}

bool Controls::lXTriggerPressed()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LTRIGGER) > 0.75;
}

bool Controls::resetArmPressed()
{
    return rJoy_.GetRawButton(InputConstants::OUTAKE_BUTTON);
}

double Controls::xboxLJoyX()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
}
double Controls::xboxLJoyY()
{
    return -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
}
double Controls::xboxRJoyX()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X);
}
double Controls::xboxRJoyY()
{
    return -xbox_.GetRawAxis(InputConstants::XBOX_RJOY_Y);
}

bool Controls::dPadUpPressed()
{
    return ((xbox_.GetPOV() < 10 && xbox_.GetPOV() >=0) || (xbox_.GetPOV() <= 360 && xbox_.GetPOV() > 350));
}
bool Controls::dPadDownPressed()
{
    return (xbox_.GetPOV() < 190 && xbox_.GetPOV() > 170);
}
bool Controls::dPadLeftPressed()
{
    return (xbox_.GetPOV() < 280 && xbox_.GetPOV() > 260);
}
bool Controls::dPadRightPressed(){
    return (xbox_.GetPOV() < 100 && xbox_.GetPOV() > 80);
}
