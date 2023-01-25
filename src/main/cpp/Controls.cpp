#include "Controls.h"

Controls::Controls() : xbox_{InputConstants::XBOX_PORT}
{
}

double Controls::getXStrafe()
{
    /*if(manuallyOverrideTurret()) //For child proofing
    {
        return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X);
    }*/

    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);
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

    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
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

    double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    if(abs(turn) < 0.05)
    {
        return 0;
    }
    turn = (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;

    return turn;
}

bool Controls::fieldOrient()
{
    return xbox_.GetRawButton(InputConstants::FIELD_ORIENT_BUTTON);
}

double Controls::getClimbPower()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_Y) * 0.5 * GeneralConstants::MAX_VOLTAGE;
}

bool Controls::getPneumatic1Toggle()
{
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC1_BUTTON);
}

bool Controls::getPneumatic2Toggle()
{
    return xbox_.GetRawButtonPressed(InputConstants::CLIMB_PNEUMATIC2_BUTTON);
}

bool Controls::autoClimbPressed()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_BUTTON);
}

bool Controls::autoClimbCancelled()
{
    return xbox_.GetRawButtonPressed(InputConstants::AUTO_CLIMB_CANCEL);
}

bool Controls::intakePressed()
{
    return rJoy_.GetTrigger();
}

bool Controls::outakePressed()
{
    return rJoy_.GetRawButton(InputConstants::OUTAKE_BUTTON);
}

bool Controls::shootPressed()
{
    return lJoy_.GetTrigger()/* && resetUnload()*/; //Second part for child proofing
}

bool Controls::increaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_UP_BUTTON);
}

bool Controls::decreaseRange()
{
    return xbox_.GetRawButtonPressed(InputConstants::DISTANCE_DOWN_BUTTON);
}

double Controls::getTurretManual()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X) * 0.2 * GeneralConstants::MAX_VOLTAGE;
}
//0,7 0
//0.8, 100
//1, 420
//2, 2280
//3, 4200
//4, 6100

bool Controls::launchpadShotDown()
{
    return xbox_.GetRawButton(InputConstants::CLIMB_PNEUMATIC1_BUTTON);
}

bool Controls::tarmacShotDown()
{
    return xbox_.GetRawButton(InputConstants::CLIMB_PNEUMATIC2_BUTTON);
}

bool Controls::resetUnload()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LTRIGGER) > 0.75;
}

bool Controls::manuallyOverrideTurret()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_RTRIGGER) > 0.75;
}

double Controls::getHoodTicks()
{
    return abs(xbox_.GetRawAxis(InputConstants::XBOX_RJOY_Y)) * ShooterConstants::MAX_HOOD_TICKS;
}

double Controls::getTurretPos()
{
    return xbox_.GetRawAxis(InputConstants::XBOX_LJOY_X) * 90;