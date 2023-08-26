#include "Intake/CubeGrabber.h"

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

bool CubeGrabber::isIntaking(){
    return grabber_status == State::INTAKING;
}

bool CubeGrabber::isOuttaking(){
    return grabber_status == State::OUTTAKING;
}

/**
 * Spins the motors such that they will intake a cube
*/
void CubeGrabber::Intake() {
    if(grabber_status == State::INTAKING){
        return;
    }
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
    if(grabber_status == State::OUTTAKING){
        return;
    }
    grabber_status = OUTTAKING;
    left_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE});
    right_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE});
}

/**
 * Reverses the motors so they will spit out the cube slowly.
*/
void CubeGrabber::OuttakeSlow() {
    if(grabber_status == State::OUTTAKING_SLOW){
        return;
    }
    grabber_status = OUTTAKING_SLOW;
    //left_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE_SLOW});
    left_motor.SetVoltage(units::volt_t{0.0}); //Weaker or more resistance - just turn off
    right_motor.SetVoltage(units::volt_t{CubeGrabberConstants::COUNTERCLOCKWISE_VOLTAGE_SLOW});
}