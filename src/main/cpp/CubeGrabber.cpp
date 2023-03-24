#include "CubeGrabber.h"

CubeGrabber::CubeGrabber() {
    left_motor.SetVoltage(units::volt_t{0});
    right_motor.SetVoltage(units::volt_t{0});
}

/**
 * Returns the current state of the cube grabber
*/
CubeGrabber::State CubeGrabber::getState() {
    return grabber_status;
}

/**
 * Spins the motors such that they will intake a cube
*/
void CubeGrabber::Intake() {
    grabber_status = INTAKING;
    left_motor.SetVoltage(units::volt_t{CubeGrabberConstants::CLOCKWISE_VOLTAGE});
    right_motor.SetVoltage(units::volt_t{CubeGrabberConstants::CLOCKWISE_VOLTAGE});
}

/**
 * Stops all motor movement
*/
void CubeGrabber::Stop() {
    grabber_status = STOPPED;
    left_motor.SetVoltage(units::volt_t{0});
    right_motor.SetVoltage(units::volt_t{0});
}

/**
 * Reverses the motors so they will spit out the cube.
*/
void CubeGrabber::Outtake() {
    grabber_status = OUTTAKING;
    left_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE});
    right_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE});
}