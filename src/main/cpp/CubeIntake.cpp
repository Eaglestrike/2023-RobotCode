#include "CubeIntake.h"

/**
 * Call this to start the deployment of the intake
*/
void CubeIntake::Deploy() {
  m_state = DEPLOYING;
  ResetEncoderPosition();
}

/**
 * Call this to start the stowing of the intake
*/
void CubeIntake::Stow() {
  m_state = STOWING;
}

/**
 * Resets arm. This includes:
 * 
 * - resetting the encoder position
 * - resetting the state so that the robot thinks the arm is stowed
 * 
 * This should be called when enabled.
*/
void CubeIntake::Reset() {
  ResetEncoderPosition();
  ResetPID();
  m_state = STOWED;
}

bool CubeIntake::isDeployed() {
  return m_state == DEPLOYED;
}

bool CubeIntake::isStowed() {
  return m_state == STOWED;
}

void CubeIntake::ResetEncoderPosition() {
  m_deployer.SetSelectedSensorPosition(0);
}

/**
 * Resets the PID
 * 
 * If using this along with ResetEncoderPosition(), call
 * ResetEncoderPosition() BEFORE ResetPID()
*/
void CubeIntake::ResetPID() {
  m_pid.Reset(units::radian_t(m_deployer.GetSelectedSensorPosition()));
}

/**
 * Important to stow intake before match starts
*/
void CubeIntake::RobotInit() {
  Reset();
}

/**
 * Code to continuously run during teleop
*/
void CubeIntake::TeleopPeriodic() {
  switch (m_state) {
    case STOWED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{0});
    case DEPLOYED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{CubeIntakeConstants::ROLLER_MAX_VOLTAGE});
  }
}

/**
 * Code to continuously run during autonomous; state machine is the same as teleop
*/
void CubeIntake::AutonomousPeriodic() {
  TeleopPeriodic();
}