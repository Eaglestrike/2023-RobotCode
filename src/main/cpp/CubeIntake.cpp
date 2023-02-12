#include "CubeIntake.h"

CubeIntake::CubeIntake() {
  m_pid.SetTolerance(units::radian_t{CubeIntakeConstants::POS_ERR_TOLERANCE},
    units::radian_t{CubeIntakeConstants::VEL_ERR_TOLERANCE});
}

/**
 * Call this to start the deployment of the intake
*/
void CubeIntake::Deploy() {
  m_state = DEPLOYING;
  ResetEncoderPosition();
  ResetPID();
  ResetAcceleration();
}

/**
 * Call this to start the stowing of the intake
*/
void CubeIntake::Stow() {
  m_state = STOWING;
  ResetPID();
  ResetAcceleration();
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
  ResetAcceleration();
  m_state = STOWED;
}

/**
 * Resets the PID
 * 
 * If using this along with ResetEncoderPosition(), call
 * ResetPID() AFTER ResetEncoderPosition()
*/
void CubeIntake::ResetPID() {
  m_pid.Reset(units::radian_t(m_deployer.GetSelectedSensorPosition()));
}

/**
 * Resets acceleration calculations
*/
void CubeIntake::ResetAcceleration() {
  m_lastTime = frc::Timer::GetFPGATimestamp();
  m_lastSpeed = units::radians_per_second_t{0};
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
void CubeIntake::Periodic() {
  switch (m_state) {
    case STOWED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{0});
    case DEPLOYED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{CubeIntakeConstants::ROLLER_MAX_VOLTAGE});
    case DEPLOYING:
      double pidVal = m_pid.Calculate(m_getEncoderRadians(), 
        m_convertStepsToRadians(CubeIntakeConstants::INTAKE_ENCODER_DISTANCE));
      auto acceleration = (m_pid.GetSetpoint().velocity - m_lastSpeed) /
        (frc::Timer::GetFPGATimestamp() - m_lastTime);
      
      m_deployer.SetVoltage(units::volt_t{pidVal});

      if (m_pid.AtGoal()) {
        m_state = DEPLOYED;
      }

      m_lastSpeed = m_pid.GetSetpoint().velocity;
      m_lastTime = frc::Timer::GetFPGATimestamp();
    case STOWING:
      double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{0});
      auto acceleration = (m_pid.GetSetpoint().velocity - m_lastSpeed) /
        (frc::Timer::GetFPGATimestamp() - m_lastTime);
      
      m_deployer.SetVoltage(units::volt_t{pidVal});

      if (m_pid.AtGoal()) {
        m_state = STOWED;
      }

      m_lastSpeed = m_pid.GetSetpoint().velocity;
      m_lastTime = frc::Timer::GetFPGATimestamp();
  }
}

units::radian_t CubeIntake::m_convertStepsToRadians(double val) {
  double valRad = val * (2 * M_PI / 2048);
  return units::radian_t{valRad};
}

units::radian_t CubeIntake::m_getEncoderRadians() {
  double pos = m_deployer.GetSelectedSensorPosition();
  return m_convertStepsToRadians(pos);
}