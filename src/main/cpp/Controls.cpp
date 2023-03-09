#include "Controls.h"

Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT}, buttonBoard_{InputConstants::BUTTON_BOARD_PORT}
{
    dPadUpDown_ = false;
    dPadLeftDown_ = false;
    dPadRightDown_ = false;
    // intakeDown_ = false;
    // outakeDown_ = false;
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

    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
    // double x = xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    if (abs(x) < 0.05)
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

    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    // double y = -xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y);
    if (abs(y) < 0.05)
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

    double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    // double turn = xbox_.GetRawAxis(InputConstants::XBOX_RJOY_X);
    if (abs(turn) < 0.05)
    {
        return 0;
    }
    turn = (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;

    return turn;
}

bool Controls::lJoyTriggerDown()
{
    return lJoy_.GetTrigger();
}

bool Controls::rJoyTriggerPressed()
{
    return rJoy_.GetTriggerPressed();
}

bool Controls::aDown()
{
    return xbox_.GetRawButton(InputConstants::A_BUTTON);
}

bool Controls::bDown()
{
    return xbox_.GetRawButton(InputConstants::B_BUTTON);
}

bool Controls::xDown()
{
    return xbox_.GetRawButton(InputConstants::X_BUTTON);
}

bool Controls::yDown()
{
    return xbox_.GetRawButton(InputConstants::Y_BUTTON);
}

bool Controls::rBumperPressed()
{
    return xbox_.GetRawButtonPressed(InputConstants::R_BUMPER);
}

bool Controls::lBumperDown()
{
    return xbox_.GetRawButton(InputConstants::L_BUMPER);
}

bool Controls::fieldOrient()
{
    return xbox_.GetRawButton(InputConstants::FIELD_ORIENT_BUTTON);
}

bool Controls::rXTriggerDown()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_RTRIGGER) > 0.75;
}

bool Controls::lXTriggerDown()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LTRIGGER) > 0.75;
}

bool Controls::outakePressed()
{
    return lJoy_.GetRawButtonPressed(InputConstants::OUTAKE_BUTTON);
}

bool Controls::intakePressed()
{
    return rJoy_.GetRawButtonPressed(InputConstants::INTAKE_BUTTON);
}

bool Controls::lLowerButtonPressed()
{
    return lJoy_.GetRawButtonPressed(InputConstants::LOWER_BUTTON);
}

bool Controls::rLowerButtonPressed()
{
    return rJoy_.GetRawButtonPressed(InputConstants::LOWER_BUTTON);
}

bool Controls::autoBalanceDown()
{
    return rJoy_.GetRawButton(InputConstants::AUTO_BALANCE_BUTTON);
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
    bool down = ((xbox_.GetPOV() < 10 && xbox_.GetPOV() >= 0) || (xbox_.GetPOV() <= 360 && xbox_.GetPOV() > 350));
    if (down && !dPadUpDown_)
    {
        dPadUpDown_ = true;
        return true;
    }
    else if (dPadUpDown_ && down)
    {
        return false;
    }
    else
    {
        dPadUpDown_ = false;
        return false;
    }


    // return ((xbox_.GetPOV() < 10 && xbox_.GetPOV() >= 0) || (xbox_.GetPOV() <= 360 && xbox_.GetPOV() > 350));
}
bool Controls::dPadDownDown()
{
    return (xbox_.GetPOV() < 190 && xbox_.GetPOV() > 170);
}
bool Controls::dPadLeftPressed()
{
    bool down = (xbox_.GetPOV() < 280 && xbox_.GetPOV() > 260);
    if (down && !dPadLeftDown_)
    {
        dPadLeftDown_ = true;
        return true;
    }
    else if (dPadLeftDown_ && down)
    {
        return false;
    }
    else
    {
        dPadLeftDown_ = false;
        return false;
    }

    // return (xbox_.GetPOV() < 280 && xbox_.GetPOV() > 260);
}
bool Controls::dPadRightPressed()
{
    bool down = (xbox_.GetPOV() < 100 && xbox_.GetPOV() > 80);
    if (down && !dPadRightDown_)
    {
        dPadRightDown_ = true;
        return true;
    }
    else if (dPadRightDown_ && down)
    {
        return false;
    }
    else
    {
        dPadRightDown_ = false;
        return false;
    }

    //return (xbox_.GetPOV() < 100 && xbox_.GetPOV() > 80);
}

bool Controls::bbUpDown()
{
    return buttonBoard_.GetRawButton(InputConstants::BB_UP);
}

bool Controls::inchingUpPressed()
{
    bool down = ((rJoy_.GetPOV() < 10 && rJoy_.GetPOV() >= 0) || (rJoy_.GetPOV() <= 360 && rJoy_.GetPOV() > 350));
    if (down && !inchingUpDown_)
    {
        inchingUpDown_ = true;
        return true;
    }
    else if (inchingUpDown_ && down)
    {
        return false;
    }
    else
    {
        inchingUpDown_ = false;
        return false;
    }
}

bool Controls::inchingDownPressed()
{
    bool down = (rJoy_.GetPOV() < 190 && rJoy_.GetPOV() > 170);
    if (down && !inchingDownDown_)
    {
        inchingDownDown_ = true;
        return true;
    }
    else if (inchingDownDown_ && down)
    {
        return false;
    }
    else
    {
        inchingDownDown_ = false;
        return false;
    }
}

bool Controls::inchingLeftPressed()
{
    bool down = (rJoy_.GetPOV() < 280 && rJoy_.GetPOV() > 260);
    if (down && !inchingLeftDown_)
    {
        inchingLeftDown_ = true;
        return true;
    }
    else if (inchingLeftDown_ && down)
    {
        return false;
    }
    else
    {
        inchingLeftDown_ = false;
        return false;
    }
}

bool Controls::inchingRightPressed()
{
    bool down = (rJoy_.GetPOV() < 100 && rJoy_.GetPOV() > 80);
    if (down && !inchingRightDown_)
    {
        inchingRightDown_ = true;
        return true;
    }
    else if (inchingRightDown_ && down)
    {
        return false;
    }
    else
    {
        inchingRightDown_ = false;
        return false;
    }
}

int Controls::checkScoringButtons()
{
    if (buttonBoard_.GetRawButton(InputConstants::B1))
    {
        return 1;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B2))
    {
        return 2;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B3))
    {
        return 3;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B4))
    {
        return 4;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B5))
    {
        return 5;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B6))
    {
        return 6;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B7))
    {
        return 7;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B8))
    {
        return 8;
    }
    else if (buttonBoard_.GetRawButton(InputConstants::B9))
    {
        return 9;
    }
    else
    {
        return -1;
    }
}

int Controls::checkLevelButtons()
{
    if(buttonBoard_.GetRawButton(InputConstants::L1))
    {
        return 1;
    }
    else if(buttonBoard_.GetRawButton(InputConstants::L2))
    {
        return 2;
    }
    else if(buttonBoard_.GetRawButton(InputConstants::L3))
    {
        return 3;
    }
    else if(buttonBoard_.GetRawButton(InputConstants::BB_DOWN))
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

int Controls::checkPSButtons()
{
    if(buttonBoard_.GetRawButton(InputConstants::BB_LEFT))
    {
        return 1;
    }
    else if(buttonBoard_.GetRawButton(InputConstants::BB_RIGHT))
    {
        return 2;
    }
    else
    {
        return -1;
    }
}