#include "MotorIntake.h"

MotorIntake::MotorIntake()
{
  m_pid.SetTolerance(units::radian_t{MotorIntakeConstants::POS_ERR_TOLERANCE},
                     units::radians_per_second_t{MotorIntakeConstants::VEL_ERR_TOLERANCE});
}

/**
 * Causes the intake to go down
 */
void MotorIntake::Deploy()
{
  m_deployerState = DEPLOYING;
  ResetPID();
  ResetAcceleration();
}

/**
 * Causes the intake to come up
 *
 * (USE Idle() or Consume() INSTEAD OF THIS TO STOW THE INTAKE)
 */
void MotorIntake::Stow()
{
  m_deployerState = STOWING;
  ResetPID();
  ResetAcceleration();
}

MotorIntake::DeployerState MotorIntake::getDeployerState()
{
  return m_deployerState;
}

MotorIntake::RollerState MotorIntake::getRollerState()
{
  return m_rollerState;
}

void MotorIntake::ResetEncoderPosition()
{
  m_deployerMotor.SetSelectedSensorPosition(0);
}

/**
 * Resets intake. This includes:
 *
 * - resetting the encoder position
 * - resetting the state so that the robot thinks the arm is stowed
 *
 * Note that this does not move any motor positions.
 *
 * This should be called on init (the RobotInit() method for this class does that for you).
 */
void MotorIntake::Reset()
{
  ResetEncoderPosition();
  ResetPID();
  ResetAcceleration();
  m_deployerState = STOWED;
  m_rollerState = STOP;
  m_intakeState = IDLE;
}

/**
 * Resets the PID
 *
 * If using this along with ResetEncoderPosition(), call
 * ResetPID() AFTER ResetEncoderPosition()
 */
void MotorIntake::ResetPID()
{
  m_pid.Reset(units::radian_t(m_deployerMotor.GetSelectedSensorPosition()));
}

/**
 * Resets acceleration calculations (currently unused; may use in the future
 * in case we need feedforward calculations)
 */
void MotorIntake::ResetAcceleration()
{
  m_lastTime = frc::Timer::GetFPGATimestamp();
  m_lastSpeed = units::radians_per_second_t{0};
}

/**
 * Important to stow intake before match starts
 */
void MotorIntake::RobotInit()
{
  Reset();
}

bool MotorIntake::m_MotorsNotNeeded()
{
  return m_intakeState == IDLE || m_intakeState == CONSUMED;
}

void MotorIntake::m_DeployerStateMachine()
{
  if (m_MotorsNotNeeded())
  {
    m_deployerMotor.SetVoltage(units::volt_t{0});
    return;
  }

  switch (m_deployerState)
  {
  case STOWED:
    m_deployerMotor.SetVoltage(units::volt_t{0});

    break;
  case DEPLOYED:
    m_deployerMotor.SetVoltage(units::volt_t{0});

    break;
  case DEPLOYING:
  {
    // calculate motion-profiled PID for deploying
    double pidVal = m_pid.Calculate(m_getEncoderRadians(),
                                    Helpers::convertStepsToRadians(MotorIntakeConstants::ENCODER_DEPLOYED_TARGET, 2048));
    double voltage = std::clamp(pidVal, -MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE, MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE);

    m_deployerMotor.SetVoltage(units::volt_t{voltage});

    if (m_pid.AtGoal())
    {
      m_deployerState = DEPLOYED;
    }

    break;
  }
  case STOWING:
  {
    // calculate motion-profiled PID for stowing
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{0});
    double voltage = std::clamp(pidVal, -MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE, MotorIntakeConstants::DEPLOYER_MAX_VOLTAGE);

    m_deployerMotor.SetVoltage(units::volt_t{voltage});

    if (m_pid.AtGoal())
    {
      m_deployerState = STOWED;
    }

    break;
  }
  }
}

void MotorIntake::m_RollerStateMachine()
{
  if (m_deployerState != DEPLOYED || m_MotorsNotNeeded() ||
      m_rollerMotor.GetSupplyCurrent() >= MotorIntakeConstants::ROLLER_STALL_CURRENT)
  {
    m_rollerMotor.SetVoltage(units::volt_t(0));
    return;
  }

  switch (m_rollerState)
  {
  case INTAKE:
    m_rollerMotor.SetVoltage(units::volt_t(MotorIntakeConstants::ROLLER_INTAKE_VOLTAGE));
  case OUTTAKE:
    m_rollerMotor.SetVoltage(units::volt_t(MotorIntakeConstants::ROLLER_OUTTAKE_VOLTAGE));
  case STOP:
    m_rollerMotor.SetVoltage(units::volt_t(0));
  default:
    m_rollerMotor.SetVoltage(units::volt_t(0));
  }
}

/**
 * Code to continuously run during auto/teleop
 */
void MotorIntake::Periodic()
{
  m_DeployerStateMachine();
  m_RollerStateMachine();

  switch (m_intakeState)
  {
  case IDLING:
    // wait until the deployer is fully stowed
    if (m_deployerState == STOWED)
    {
      m_intakeState = IDLE;
    }
  case CONSUMING:
  {
    // wait for the deployer to deploy
    // then wait for cone to be in deployer
    // then wait for deployer to stow the cone
    if (m_deployerState == DEPLOYED)
    {
      if (m_rollerMotor.GetSupplyCurrent() >= MotorIntakeConstants::ROLLER_STALL_CURRENT)
      {
        m_deployerState = STOWING;
        m_rollerState = STOP;
      }
    }
    if (m_deployerState == STOWED)
    {
      m_intakeState = CONSUMED;
    }
  }
  case SPITTING:
  {
    // wait for deployer to deploy
    // then roll motors
    if (m_deployerState == DEPLOYED)
    {
      m_intakeState = INTAKE_DOWN;
    }
  }
  default:
    m_rollerState = STOP;
  }
}

/**
 * Call this to start the cone intaking procedure.
 */
void MotorIntake::Consume()
{
  m_intakeState = CONSUMING;
  m_rollerState = INTAKE;
  m_deployerState = DEPLOYING;
}

/**
 * Call this to start the cone outtaking procedure for the arm to pick up
 * (or for potentially unjamming the cone).
 */
void MotorIntake::Spit()
{
  m_intakeState = SPITTING;
  m_rollerState = OUTTAKE;
  m_deployerState = DEPLOYING;
}

/**
 * Call this to store the intake with no cone inside.
 */
void MotorIntake::Idle()
{
  m_intakeState = IDLING;
  m_deployerState = STOWING;
  m_rollerState = STOP;
}

/**
 * If the cone is currently stored in the robot.
 */
bool MotorIntake::IsStored()
{
  return m_intakeState == CONSUMED;
}

/**
 * If the cone intake is stowed without a cone inside.
 */
bool MotorIntake::IsIdle()
{
  return m_intakeState == IDLE;
}

/**
 * If the intake is down and currently spitting out the cone.
 *
 * This method can be used to detect if the cone is ready to intake;
 * however, a delay might be needed
 *
 * TODO Test delay
 */
bool MotorIntake::IntakeDown()
{
  return m_intakeState == INTAKE_DOWN;
}

units::radian_t MotorIntake::m_getEncoderRadians()
{
  double pos = m_deployerMotor.GetSelectedSensorPosition();
  return Helpers::convertStepsToRadians(pos, 2048);
}
