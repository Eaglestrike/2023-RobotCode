#include "MotorIntake.h"

/**
 * Constructor
 *
 * @param showSmartDashboard Set true if state machines should be shown to SmartDashboard
 * @param showConstants If constants should be shown and set.
 * @param debug If debug smartdashboard should be shown.
 */
MotorIntake::MotorIntake(bool showSmartDashboard, bool showConstants, bool showDebug)
    : m_showSmartDashboard{showSmartDashboard}, m_showConstants{showConstants}, m_showDebug{showDebug}
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
  m_PutCurrentConeIntakeState();
  m_PutCurrentDeployerState();
  m_PutCurrentRollerState();
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
 * If the arm can pass through the cone intake.
 *
 * @warning Use this method before moving the arm!!
 */
bool MotorIntake::IsClearForArm()
{
  return m_deployerState == GROUND;
}

/**
 * Causes the intake to go down and stops rollers
 */
void MotorIntake::Ground()
{
  m_deployerState = GROUNDING;
  m_rollerState = STOP;
  ResetPID();
  ResetAcceleration();
}

/**
 * Causes intake to go to middle position to prepare for cone to be grabbed
 *
 * Also stops the rollers
 */
void MotorIntake::Middle()
{
  m_deployerState = MIDDLING;
  m_rollerState = STOP;
  ResetPID();
  ResetAcceleration();
}

/**
 * Causes the intake to come up and stops rollers
 */
void MotorIntake::Stow()
{
  ResetPID();
  ResetAcceleration();
  m_deployerState = STOWING;
  m_rollerState = STOP;
}

/**
 * Deploys and outtakes cone
 */
void MotorIntake::Spit()
{
  ResetPID();
  ResetAcceleration();
  m_deployerState = GROUNDING;
  m_rollerState = OUTTAKE;
}

/**
 * Waits for cone on the ground.
 *
 * Deploys the intake and then spins the rollers for cone to come in.
 */
void MotorIntake::WaitForCone()
{
  ResetPID();
  ResetAcceleration();
  m_deployerState = GROUNDING;
  m_rollerState = INTAKE;
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
 * Resets encoder position.
 */
void MotorIntake::ResetEncoderPosition()
{
  // m_deployerMotorEncoder.SetPosition(0);
  m_deployerMotor.SetSelectedSensorPosition(0);
}

/**
 * Resets intake. This includes:
 *
 * - resetting the encoder position
 * - resetting the state so that the robot thinks the intake is stowed
 *
 * @note This does not move any motor positions (idk what past me meant by this).
 * @note Call this when cone intake is up (in stowed position)
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
  // m_pid.Reset(units::radian_t(m_deployerMotorEncoder.GetPosition()));
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
 * Puts constants to SmartDashboard if showConstants is set to true.
 */
void MotorIntake::PutConstants()
{
  if (!m_showConstants)
  {
    return;
  }

  frc::SmartDashboard::PutNumber("Cone Intake Ground Tgt", m_groundGoal);
  frc::SmartDashboard::PutNumber("Cone Intake Middle Tgt", m_middleGoal);
  frc::SmartDashboard::PutNumber("Cone Intake Stowed Tgt", m_stowedGoal);
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

  m_groundGoal = frc::SmartDashboard::GetNumber("Cone Intake Ground Tgt", m_groundGoal);
  m_middleGoal = frc::SmartDashboard::GetNumber("Cone Intake Middle Tgt", m_middleGoal);
  m_stowedGoal = frc::SmartDashboard::GetNumber("Cone Intake Stowed Tgt", m_stowedGoal);
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
 * Put debug info on smartdashboard if showDebug is true
 */
void MotorIntake::PutDebug()
{
  if (!m_showDebug)
  {
    return;
  }

  // frc::SmartDashboard::PutNumber("Cone Intake Encoder", m_deployerMotorEncoder.GetCountsPerRevolution());
  frc::SmartDashboard::PutNumber("Cone Intake Encoder", m_deployerMotor.GetSelectedSensorPosition());
}

void MotorIntake::m_DeployerStateMachine()
{
  switch (m_deployerState)
  {
  case STOWED:
    // still apply pid if stowed so it is not moved/disturbed
  case STOWING:
  {
    // calculate motion-profiled PID for stowing
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_stowedGoal});
    double voltage = std::clamp(pidVal, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake curpos", m_getEncoderRadians().value());
      frc::SmartDashboard::PutNumber("Cone Intake targetpos", m_stowedGoal);
      frc::SmartDashboard::PutNumber("Cone Intake current kP", m_pid.GetP());
      frc::SmartDashboard::PutNumber("Cone Intake V to motor", voltage);
    }

    m_deployerMotor.SetVoltage(units::volt_t(voltage));

    if (m_pid.AtGoal())
    {
      m_deployerState = STOWED;
    }

    break;
  }
  case GROUND:
    // still apply pid if on the ground so it is not moved/disturbed
  case GROUNDING:
  {
    // calculate motion-profiled PID for deploying
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_groundGoal});
    double voltage = std::clamp(pidVal, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake curpos", m_getEncoderRadians().value());
      frc::SmartDashboard::PutNumber("Cone Intake targetpos", m_groundGoal);
      frc::SmartDashboard::PutNumber("Cone Intake current kP", m_pid.GetP());
      frc::SmartDashboard::PutNumber("Cone Intake V to motor", voltage);
    }

    m_deployerMotor.SetVoltage(units::volt_t(voltage));

    if (m_pid.AtGoal())
    {
      m_deployerState = GROUND;
    }
    break;
  }
  case MIDDLE:
    // still apply pid if in the middle so it is not moved/disturbed
  case MIDDLING:
  {
    // calculate motion-profiled PID for stowing
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_middleGoal});
    double voltage = std::clamp(pidVal, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake curpos", m_getEncoderRadians().value());
      frc::SmartDashboard::PutNumber("Cone Intake targetpos", m_middleGoal);
      frc::SmartDashboard::PutNumber("Cone Intake current kP", m_pid.GetP());
      frc::SmartDashboard::PutNumber("Cone Intake V to motor", voltage);
    }

    m_deployerMotor.SetVoltage(units::volt_t(voltage));

    if (m_pid.AtGoal())
    {
      m_deployerState = STOWED;
    }

    break;
  }
  default:
    m_deployerMotor.SetVoltage(units::volt_t{0});
  }
}

void MotorIntake::m_RollerStateMachine()
{
  if (m_rollerMotor.GetSupplyCurrent() >= m_rollerStallCurrent)
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
  double pos = m_deployerMotor.GetSelectedSensorPosition();
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
  case GROUND:
    stateStr = "On ground";
    break;
  case STOWING:
    stateStr = "Stowing";
    break;
  case GROUNDING:
    stateStr = "Going to ground";
    break;
  case MIDDLING:
    stateStr = "Going to intake";
    break;
  case MIDDLE:
    stateStr = "Intake pos";
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

/**
 * Call this to start the cone intaking procedure.
 *
 * @deprecated do not use
 */
void MotorIntake::Consume()
{
  m_intakeState = CONSUMING;
  m_rollerState = INTAKE;
  m_deployerState = GROUNDING;
}

/**
 * Call this to store the intake with no cone inside.
 *
 * Functionally does the same as Stow()
 *
 * @deprecated do not use
 */
void MotorIntake::Idle()
{
  m_intakeState = IDLING;
  m_deployerState = STOWING;
  m_rollerState = STOP;
}

/**
 * If the cone is currently stored in the robot.
 *
 * @deprecated do not use
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
 *
 * @deprecated do not use
 */
bool MotorIntake::IntakeDown()
{
  return m_intakeState == INTAKE_DOWN;
}

/**
 * If the cone intake is stowed without a cone inside.
 *
 * @deprecated do not use
 */
bool MotorIntake::IsIdle()
{
  return m_intakeState == IDLE;
}
