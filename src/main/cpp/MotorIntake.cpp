#include "MotorIntake.h"

/**
 * Constructor
 *
 * @param showSmartDashboard Set true if state machines should be shown to SmartDashboard
 */
MotorIntake::MotorIntake(bool showSmartDashboard, bool showConstants)
    : m_showSmartDashboard{showSmartDashboard}, m_showConstants{showConstants}
{
  m_pid.SetTolerance(units::radian_t{MotorIntakeConstants::POS_ERR_TOLERANCE},
                     units::radians_per_second_t{MotorIntakeConstants::VEL_ERR_TOLERANCE});
}

/**
 * Important to stow intake before match starts
 */
void MotorIntake::RobotInit()
{
  Reset();
  PutConstants();
}

/**
 * Code to continuously run during auto/teleop
 */
void MotorIntake::Periodic()
{
  m_PutCurrentConeIntakeState();
  m_PutCurrentDeployerState();
  m_PutCurrentRollerState();

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

    break;
  case CONSUMING:
  {
    // wait for the deployer to deploy
    if (m_deployerState == DEPLOYED)
    {
      // wait for cone to be in deployer
      if (m_rollerMotor.GetSupplyCurrent() >= m_rollerStallCurrent)
      {
        m_deployerState = STOWING;
        m_rollerState = STOP;
      }
    }
    // once cone is in deployer, wait for deployer to stow the cone
    if (m_deployerState == STOWED)
    {
      m_intakeState = CONSUMED;
    }

    break;
  }
  case SPITTING:
  {
    // wait for deployer to deploy
    // then roll motors
    if (m_deployerState == DEPLOYED)
    {
      m_intakeState = INTAKE_DOWN;
    }

    break;
  }
  default:
    m_rollerState = STOP;
  }
}

/**
 * Disabled code (stops both motors)
 */
void MotorIntake::DisabledInit()
{
  m_rollerMotor.SetVoltage(units::volt_t{0});
  m_deployerMotor.SetVoltage(units::volt_t{0});
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
 *
 * Functionally does the same as Stow()
 */
void MotorIntake::Idle()
{
  m_intakeState = IDLING;
  m_deployerState = STOWING;
  m_rollerState = STOP;
}

/**
 * If the arm can pass through the cone intake.
 *
 * @warning Use this method before moving the arm!!
 */
bool MotorIntake::IsClearForArm()
{
  return m_deployerState == STOWED || m_deployerState == DEPLOYED;
}

/**
 * If the cone is currently stored in the robot.
 */
bool MotorIntake::IsStored()
{
  return m_intakeState == CONSUMED;
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

/**
 * If the cone intake is stowed without a cone inside.
 */
bool MotorIntake::IsIdle()
{
  return m_intakeState == IDLE;
}

MotorIntake::DeployerState MotorIntake::getDeployerState()
{
  return m_deployerState;
}

MotorIntake::RollerState MotorIntake::getRollerState()
{
  return m_rollerState;
}

/**
 * Puts constants to SmartDashboard if showConstants is set to true.
 */
void MotorIntake::PutConstants()
{
  if (!m_showConstants)
  {
    return;
  }

  frc::SmartDashboard::PutNumber("Cone Intake Target", m_deployerGoal);
  frc::SmartDashboard::PutNumber("Cone Intake Deployer Max V", m_deployerMaxVoltage);
  frc::SmartDashboard::PutNumber("Cone Intake Roller Intake V", m_rollerIntakeVoltage);
  frc::SmartDashboard::PutNumber("Cone Intake Roller Outtake V", m_rollerOuttakeVoltage);
  frc::SmartDashboard::PutNumber("Cone Intake Roller Stall I", m_rollerStallCurrent);
  frc::SmartDashboard::PutNumber("Cone Intake kP", m_kP);
  frc::SmartDashboard::PutNumber("Cone Intake kI", m_kI);
  frc::SmartDashboard::PutNumber("Cone Intake kD", m_kD);
  frc::SmartDashboard::PutNumber("Cone Intake Max Vel", m_maxVel);
  frc::SmartDashboard::PutNumber("Cone Intake Max Acc", m_maxAcc);
  frc::SmartDashboard::PutNumber("Cone Intake Pos Tol", m_posErrTolerance);
  frc::SmartDashboard::PutNumber("Cone Intake Vel Tol", m_velErrTolerance);
}

/**
 * Sets constants as variables if showConstants is set to true.
 */
void MotorIntake::SetConstants()
{
  if (!m_showConstants)
  {
    return;
  }

  m_deployerGoal = frc::SmartDashboard::GetNumber("Cone Intake Target", m_deployerGoal);
  m_deployerMaxVoltage = frc::SmartDashboard::GetNumber("Cone Intake Deployer Max V", m_deployerMaxVoltage);
  m_rollerIntakeVoltage = frc::SmartDashboard::GetNumber("Cone Intake Roller Intake V", m_rollerIntakeVoltage);
  m_rollerOuttakeVoltage = frc::SmartDashboard::GetNumber("Cone Intake Roller Outtake V", m_rollerOuttakeVoltage);
  m_rollerStallCurrent = frc::SmartDashboard::GetNumber("Cone Intake Roller Stall I", m_rollerStallCurrent);
  m_kP = frc::SmartDashboard::GetNumber("Cone Intake kP", m_kP);
  m_kI = frc::SmartDashboard::GetNumber("Cone Intake kI", m_kI);
  m_kD = frc::SmartDashboard::GetNumber("Cone Intake kD", m_kD);
  m_maxVel = frc::SmartDashboard::GetNumber("Cone Intake Max Vel", m_maxVel);
  m_maxAcc = frc::SmartDashboard::GetNumber("Cone Intake Max Acc", m_maxAcc);
  m_posErrTolerance = frc::SmartDashboard::GetNumber("Cone Intake Pos Tol", m_posErrTolerance);
  m_velErrTolerance = frc::SmartDashboard::GetNumber("Cone Intake Vel Tol", m_velErrTolerance);

  m_constraints.maxVelocity = units::radians_per_second_t{m_maxVel};
  m_constraints.maxAcceleration = units::radians_per_second_squared_t{m_maxAcc};
  m_pid.SetConstraints(m_constraints);
  m_pid.SetTolerance(units::radian_t{m_posErrTolerance}, units::radians_per_second_t{m_velErrTolerance});
  m_pid.SetPID(m_kP, m_kI, m_kD);
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
 */
void MotorIntake::Stow()
{
  m_deployerState = STOWING;
  ResetPID();
  ResetAcceleration();
}

void MotorIntake::ResetEncoderPosition()
{
  m_deployerMotor.GetEncoder().SetPosition(0);
}

/**
 * Resets intake. This includes:
 *
 * - resetting the encoder position
 * - resetting the state so that the robot thinks the intake is stowed
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
  ResetStates();
}

/**
 * Resets the state machines of the intake, deployer, and roller
 */
void MotorIntake::ResetStates()
{
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
  m_pid.Reset(units::radian_t(m_deployerMotor.GetEncoder().GetPosition()));
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
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_deployerGoal});
    double voltage = std::clamp(pidVal, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    m_deployerMotor.SetVoltage(units::volt_t(voltage));

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
    double voltage = std::clamp(pidVal, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    m_deployerMotor.SetVoltage(units::volt_t(voltage));

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
      m_rollerMotor.GetSupplyCurrent() >= m_rollerStallCurrent)
  {
    m_rollerMotor.SetVoltage(units::volt_t(0));
    return;
  }

  switch (m_rollerState)
  {
  case INTAKE:
    m_rollerMotor.SetVoltage(units::volt_t(m_rollerIntakeVoltage));
    break;
  case OUTTAKE:
    m_rollerMotor.SetVoltage(units::volt_t(m_rollerOuttakeVoltage));
    break;
  case STOP:
    m_rollerMotor.SetVoltage(units::volt_t(0));
    break;
  default:
    m_rollerMotor.SetVoltage(units::volt_t(0));
  }
}

units::radian_t MotorIntake::m_getEncoderRadians()
{
  double pos = m_deployerMotor.GetEncoder().GetPosition();
  return Helpers::convertStepsToRadians(pos, MotorIntakeConstants::DEPLOYER_STEPS_PER_REV);
}

void MotorIntake::m_PutCurrentDeployerState()
{
  if (!m_showSmartDashboard)
  {
    return;
  }

  std::string stateStr;
  switch (m_deployerState)
  {
  case STOWED:
    stateStr = "Stowed";
    break;
  case DEPLOYED:
    stateStr = "Deployed";
    break;
  case STOWING:
    stateStr = "Stowing";
    break;
  case DEPLOYING:
    stateStr = "Deploying";
    break;
  default:
    // this shouldn't happen. If it does it's problematic
    stateStr = "UNKNOWN";
  }

  frc::SmartDashboard::PutString("Cone Intake Deployer", stateStr);
}

void MotorIntake::m_PutCurrentRollerState()
{
  if (!m_showSmartDashboard)
  {
    return;
  }

  std::string stateStr;
  switch (m_rollerState)
  {
  case INTAKE:
    stateStr = "Intake";
    break;
  case OUTTAKE:
    stateStr = "Outtake";
    break;
  case STOP:
    stateStr = "Stop";
    break;
  default:
    // this shouldn't happen. If it does it's problematic
    stateStr = "UNKNOWN";
  }

  frc::SmartDashboard::PutString("Cone Intake Roller", stateStr);
}

void MotorIntake::m_PutCurrentConeIntakeState()
{
  if (!m_showSmartDashboard)
  {
    return;
  }

  std::string stateStr;
  switch (m_intakeState)
  {
  case IDLE:
    stateStr = "Idle (stowed)";
    break;
  case IDLING:
    stateStr = "Idling (stowing)";
    break;
  case CONSUMING:
    stateStr = "Intaking";
    break;
  case CONSUMED:
    stateStr = "Stored";
    break;
  case SPITTING:
    stateStr = "Spitting";
    break;
  case INTAKE_DOWN:
    stateStr = "Cone Is Outside";
    break;
  default:
    // this shouldn't happen. If it does it's problematic
    stateStr = "UNKNOWN";
  }

  frc::SmartDashboard::PutString("Cone Intake", stateStr);
}
