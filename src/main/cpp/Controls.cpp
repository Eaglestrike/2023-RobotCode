#include "Controls.h"

/**
 * Initializes left/right joysticks & xbox controller
*/
Controls::Controls() : lJoy_{InputConstants::LJOY_PORT}, rJoy_{InputConstants::RJOY_PORT}, xbox_{InputConstants::XBOX_PORT} {}

/**
 * @returns desired swerve "velocity" in the x direction as input from joystick
*/
double Controls::getXStrafe() {
    double x = lJoy_.GetRawAxis(InputConstants::LJOY_X);

    if(abs(x) < 0.05) return 0; //deadband
    x = (x > 0) ? (x - 0.05) / 0.95 : (x + 0.05) / 0.95;
    
    return x;
}

/**
 * @returns desired swerve "velocity" in the y direction as input from joystick
*/
double Controls::getYStrafe() {
    double y = -lJoy_.GetRawAxis(InputConstants::LJOY_Y);
    
    if(abs(y) < 0.05) return 0; //deadband
    y = (y > 0) ? (y - 0.05) / 0.95 : (y + 0.05) / 0.95;
    
    return y;
}

/**
 * @returns desired swerve rotational "velocity" as input from joystick
*/
double Controls::getTurn() {
    double turn = rJoy_.GetRawAxis(InputConstants::RJOY_X);
    
    if(abs(turn) < 0.05) return 0; //deadband
    turn = (turn > 0) ? ((turn - 0.05) / 0.95) * 0.3 : ((turn + 0.05) / 0.95) * 0.3;

    return turn;
}