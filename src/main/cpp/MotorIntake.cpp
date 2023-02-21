#include "MotorIntake.h"

MotorIntake::MotorIntake() {
  m_pid.SetTolerance(units::radian_t{MotorIntakeConstants::POS_ERR_TOLERANCE},
    units::radians_per_second_t{MotorIntakeConstants::VEL_ERR_TOLERANCE});
}

/**
 * Call this to start the deployment of the intake
*/
void MotorIntake::Deploy() {
  m_state = DEPLOYING;
  ResetPID();
  ResetAcceleration();
}

/**
 * Call this to start the stowing of the intake
*/
void MotorIntake::Stow() {
  m_state = STOWING;
  ResetPID();
  ResetAcceleration();
}

MotorIntake::State MotorIntake::getState() {
  return m_state;
}

void MotorIntake::ResetEncoderPosition() {
  m_deployer.SetSelectedSensorPosition(0);
}

/**
 * Resets intake. This includes:
 * 
 * - resetting the encoder position
 * - resetting the state so that the robot thinks the arm is stowed
 * 
 * Note that this does not move any motor positions.
 * 
 * This should be called on init.
*/
void MotorIntake::Reset() {
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
void MotorIntake::ResetPID() {
  m_pid.Reset(units::radian_t(m_deployer.GetSelectedSensorPosition()));
}

/**
 * Resets acceleration calculations (currently unused; may use in the future
 * in case we need feedforward calculations)
*/
void MotorIntake::ResetAcceleration() {
  m_lastTime = frc::Timer::GetFPGATimestamp();
  m_lastSpeed = units::radians_per_second_t{0};
}

/**
 * Important to stow intake before match starts
*/
void MotorIntake::RobotInit() {
  Reset();
}

/**
 * Code to continuously run during auto/teleop
*/
void MotorIntake::Periodic() {
  switch (m_state) {
    case STOWED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{0});

      break;
    case DEPLOYED:
      m_deployer.SetVoltage(units::volt_t{0});
      m_roller.SetVoltage(units::volt_t{MotorIntakeConstants::ROLLER_MAX_VOLTAGE});

      break;
    case DEPLOYING:
    {
      double pidVal = m_pid.Calculate(m_getEncoderRadians(), 
        Helpers::convertStepsToRadians(MotorIntakeConstants::ENCODER_DEPLOYED_TARGET, 2048)); 
      double voltage = std::clamp(pidVal, -MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE, MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE);

      m_deployer.SetVoltage(units::volt_t{voltage});

      if (m_pid.AtGoal()) {
        m_state = DEPLOYED;
      }

      break;
    }
    case STOWING:
    {
      double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{0}); 
      double voltage = std::clamp(pidVal, -MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE, MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE);

      m_deployer.SetVoltage(units::volt_t{voltage});

      if (m_pid.AtGoal()) {
        m_state = STOWED;
      }

      break;
    }
  }
}

units::radian_t MotorIntake::m_getEncoderRadians() {
  double pos = m_deployer.GetSelectedSensorPosition();
  return Helpers::convertStepsToRadians(pos, 2048);
}
