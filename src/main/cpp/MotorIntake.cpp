#include "MotorIntake.h"

#include <iostream>

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
  // m_PutCurrentConeIntakeState();
  m_PutCurrentDeployerState();
  m_PutCurrentRollerState();
}

/**
 * Code to continuously run during auto/teleop
 */
void MotorIntake::Periodic()
{
  // m_PutCurrentConeIntakeState();
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
 * Causes the intake to go down and stops rollers
 */
void MotorIntake::Ground()
{
  m_rollerState = STOP;
  if (m_deployerState != GROUND && m_deployerState != GROUNDING)
  {
    m_deployerState = GROUNDING;
    ResetPID();
    ResetAcceleration();
  }
}

/**
 * Causes intake to go to middle position to prepare for cone to be grabbed
 *
 * Also stops the rollers
 */
void MotorIntake::Middle()
{
  m_rollerState = STOP;
  if (m_deployerState != MIDDLE && m_deployerState != MIDDLING)
  {
    m_deployerState = MIDDLING;
    ResetPID();
    ResetAcceleration();
  }
}

/**
 * Causes the intake to come up and stops rollers
 */
void MotorIntake::Stow()
{
  m_rollerState = STOP;
  if (m_deployerState != STOWED && m_deployerState != STOWING)
  {
    m_deployerState = STOWING;
    ResetPID();
    ResetAcceleration();
  }
}

/**
 * Deploys and outtakes cone
 */
void MotorIntake::Spit()
{
  m_rollerState = OUTTAKE;
  if (m_deployerState != GROUND && m_deployerState != GROUNDING)
  {
    m_deployerState = GROUNDING;
    ResetPID();
    ResetAcceleration();
  }
}

/**
 * Waits for cone on the ground.
 *
 * Deploys the intake and then spins the rollers for cone to come in.
 */
void MotorIntake::WaitForCone()
{
  if (m_deployerState != GROUND && m_deployerState != GROUNDING)
  {
    m_deployerState = GROUNDING;
    ResetPID();
    ResetAcceleration();
  }

  if (m_rollerState != MAINTAIN)
  {
    m_rollerState = INTAKE;
  }
}

/**
 * Hands off to arm
 *
 * Stows the intake while spinning the rollers outward
 */
void MotorIntake::HandoffToArm()
{
  if (m_deployerState != STOWED && m_deployerState != STOWING)
  {
    m_deployerState = STOWING;
    ResetPID();
    ResetAcceleration();
  }
  m_rollerState = OUTTAKE;
}

/**
 * If the arm can pass through the cone intake.
 *
 * @warning Use this method before moving the arm!!
 */
bool MotorIntake::IsClearForArm()
{
  return m_deployerState == GROUND && m_getEncoderRadians().value() >= MotorIntakeConstants::GROUND_ARM_TOLERANCE_POS;
}

/**
 * If intake is ready for handoff
 *
 * @warning Use this method before calling HandoffToArm()
 */
bool MotorIntake::IsReadyForHandoff()
{
  return m_deployerState == MIDDLE;
}

/**
 * Gets deployer state
 */
MotorIntake::DeployerState MotorIntake::getDeployerState()
{
  return m_deployerState;
}

/**
 * Gets roller state
 */
MotorIntake::RollerState MotorIntake::getRollerState()
{
  return m_rollerState;
}

/**
 * Resets encoder position.
 */
void MotorIntake::ResetEncoderPosition()
{
  m_deployerMotorEncoder.SetPosition(0);
  // m_deployerMotor.SetSelectedSensorPosition(0);
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
  m_pid.Reset(units::radian_t(m_deployerMotorEncoder.GetPosition()));
  // m_pid.Reset(units::radian_t(m_deployerMotor.GetSelectedSensorPosition()));
}

/**
 * Resets acceleration calculations
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
  frc::SmartDashboard::PutNumber("Cone Intake kS", m_kS);
  frc::SmartDashboard::PutNumber("Cone Intake kV", m_kV);
  frc::SmartDashboard::PutNumber("Cone Intake kA", m_kA);
  frc::SmartDashboard::PutNumber("Cone Intake kG", m_kG);
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
  m_kS = frc::SmartDashboard::GetNumber("Cone Intake kS", m_kS);
  m_kV = frc::SmartDashboard::GetNumber("Cone Intake kV", m_kV);
  m_kA = frc::SmartDashboard::GetNumber("Cone Intake kA", m_kA);
  m_kG = frc::SmartDashboard::GetNumber("Cone Intake kG", m_kG);
  m_maxVel = frc::SmartDashboard::GetNumber("Cone Intake Max Vel", m_maxVel);
  m_maxAcc = frc::SmartDashboard::GetNumber("Cone Intake Max Acc", m_maxAcc);
  m_posErrTolerance = frc::SmartDashboard::GetNumber("Cone Intake Pos Tol", m_posErrTolerance);
  m_velErrTolerance = frc::SmartDashboard::GetNumber("Cone Intake Vel Tol", m_velErrTolerance);

  m_constraints.maxVelocity = units::radians_per_second_t{m_maxVel};
  m_constraints.maxAcceleration = units::radians_per_second_squared_t{m_maxAcc};
  m_pid.SetConstraints(m_constraints);
  m_pid.SetTolerance(units::radian_t{m_posErrTolerance}, units::radians_per_second_t{m_velErrTolerance});
  m_pid.SetPID(m_kP, m_kI, m_kD);

  m_feedForward.kS = units::volt_t{m_kS};
  m_feedForward.kG = units::volt_t{m_kG};
  m_feedForward.kA = kAUnit_t{m_kA};
  m_feedForward.kV = kVUnit_t{m_kV};
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
  // frc::SmartDashboard::PutNumber("Cone Intake Encoder", m_deployerMotor.GetSelectedSensorPosition());
}

void MotorIntake::m_DeployerStateMachine()
{
  switch (m_deployerState)
  {
  case STOWED:
    // still apply pid if stowed so it is not moved/disturbed
    m_rollerState = STOP;
    // fall through
  case STOWING:
  {
    // calculate motion-profiled PID for stowing
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_stowedGoal});
    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake pidVal", pidVal);
    }

    // feedforward calculations
    auto acceleration = (m_pid.GetSetpoint().velocity - m_lastSpeed) /
                        (frc::Timer::GetFPGATimestamp() - m_lastTime);
    auto ffOut = m_feedForward.Calculate(m_pid.GetSetpoint().position, m_pid.GetSetpoint().velocity, acceleration);
    double unclamped = pidVal + ffOut.value();
    double voltage = std::clamp(unclamped, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    double vel = m_deployerMotorEncoder.GetVelocity() * 2.0 * M_PI * MotorIntakeConstants::DEPLOYER_STEPS_PER_REV * (1 / 60.0);
    if (m_pid.GetSetpoint() == m_pid.GetGoal() && !m_AtGoal(m_stowedGoal, m_getEncoderRadians().value(), vel))
    {
      // if the profile is done but there is still an error, regenerate another profile to correct that error.
      m_deployerState = STOWING; // reset state so it thinks it's not stowed if moved out of stowed position
      m_pid.Reset(m_getEncoderRadians(), units::radians_per_second_t{vel});
    }

    m_lastSpeed = m_pid.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake current kS", m_feedForward.kS.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kV", m_feedForward.kV.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kG", m_feedForward.kG.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kA", m_feedForward.kA.value());
      frc::SmartDashboard::PutNumber("Cone Intake unclamped", unclamped);
      frc::SmartDashboard::PutNumber("Cone Intake volt", voltage);
      frc::SmartDashboard::PutNumber("Cone Intake Target", m_stowedGoal);
      frc::SmartDashboard::PutNumber("Cone Intake pos error", m_pid.GetPositionError().value());
      frc::SmartDashboard::PutNumber("Cone Intake vel error", m_pid.GetVelocityError().value());
      frc::SmartDashboard::PutNumber("FFout", ffOut.value());
    }

    std::cout << "setting for stow: " << voltage << std::endl;
    m_SetDeployerVoltage(voltage);

    if (m_AtGoal(m_stowedGoal, m_getEncoderRadians().value(), vel))
    {
      m_deployerState = STOWED;
    }

    break;
  }
  case GROUND:
    // still apply pid if on the ground so it is not moved/disturbed
    // fall through
  case GROUNDING:
  {
    // calculate motion-profiled PID for grounding
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_groundGoal});
    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake pidVal", pidVal);
    }

    // feedfoward calculations
    auto acceleration = (m_pid.GetSetpoint().velocity - m_lastSpeed) /
                        (frc::Timer::GetFPGATimestamp() - m_lastTime);
    auto ffOut = m_feedForward.Calculate(m_pid.GetSetpoint().position, m_pid.GetSetpoint().velocity, acceleration);
    double unclamped = pidVal + ffOut.value();
    double voltage = std::clamp(unclamped, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    double vel = m_deployerMotorEncoder.GetVelocity() * 2.0 * M_PI * MotorIntakeConstants::DEPLOYER_STEPS_PER_REV * (1 / 60.0);
    if (m_pid.GetSetpoint() == m_pid.GetGoal() && !m_AtGoal(m_groundGoal, m_getEncoderRadians().value(), vel))
    {
      // if the profile is done but there is still an error, regenerate another profile to correct that error.
      m_deployerState = GROUNDING; // reset state so it thinks it's not on the ground if moved out of ground position
      m_pid.Reset(m_getEncoderRadians(), units::radians_per_second_t{vel});
    }

    m_lastSpeed = m_pid.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake targetpos", m_groundGoal);
      frc::SmartDashboard::PutNumber("Cone Intake current kP", m_pid.GetP());
      frc::SmartDashboard::PutNumber("Cone Intake current kS", m_feedForward.kS.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kV", m_feedForward.kV.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kG", m_feedForward.kG.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kA", m_feedForward.kA.value());
      frc::SmartDashboard::PutNumber("Cone Intake pos error", m_pid.GetPositionError().value());
      frc::SmartDashboard::PutNumber("Cone Intake vel error", m_pid.GetVelocityError().value());
      frc::SmartDashboard::PutNumber("FFout", ffOut.value());
    }

    // std::cout << "setting from ground" << std::endl;
    m_SetDeployerVoltage(voltage);

    if (m_AtGoal(m_groundGoal, m_getEncoderRadians().value(), vel))
    {
      m_deployerState = GROUND;
    }
    break;
  }
  case MIDDLE:
    // still apply pid if in the middle so it is not moved/disturbed
    // fall through
  case MIDDLING:
  {
    // calculate motion-profiled PID for middling
    double pidVal = m_pid.Calculate(m_getEncoderRadians(), units::radian_t{m_middleGoal});
    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake pidVal", pidVal);
    }

    auto acceleration = (m_pid.GetSetpoint().velocity - m_lastSpeed) /
                        (frc::Timer::GetFPGATimestamp() - m_lastTime);
    auto ffOut = m_feedForward.Calculate(m_pid.GetSetpoint().position, m_pid.GetSetpoint().velocity, acceleration);
    double unclamped = pidVal + ffOut.value();
    double voltage = std::clamp(unclamped, -m_deployerMaxVoltage, m_deployerMaxVoltage);

    double vel = m_deployerMotorEncoder.GetVelocity() * 2.0 * M_PI * MotorIntakeConstants::DEPLOYER_STEPS_PER_REV * (1 / 60.0);
    if (m_pid.GetSetpoint() == m_pid.GetGoal() && !m_AtGoal(m_middleGoal, m_getEncoderRadians().value(), vel))
    {
      // if the profile is done but there is still an error, regenerate another profile to correct that error.
      m_deployerState = MIDDLING; // reset state so it thinks it's not in the middle if moved out of middle position
      m_pid.Reset(m_getEncoderRadians(), units::radians_per_second_t{vel});
    }

    m_lastSpeed = m_pid.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();

    if (m_showDebug)
    {
      frc::SmartDashboard::PutNumber("Cone Intake targetpos", m_middleGoal);
      frc::SmartDashboard::PutNumber("Cone Intake current kP", m_pid.GetP());
      frc::SmartDashboard::PutNumber("Cone Intake current kS", m_feedForward.kS.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kV", m_feedForward.kV.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kG", m_feedForward.kG.value());
      frc::SmartDashboard::PutNumber("Cone Intake current kA", m_feedForward.kA.value());
      frc::SmartDashboard::PutNumber("Cone Intake pos error", m_pid.GetPositionError().value());
      frc::SmartDashboard::PutNumber("Cone Intake vel error", m_pid.GetVelocityError().value());
      frc::SmartDashboard::PutNumber("FFout", ffOut.value());
    }

    // std::cout << "setting from middle" << std::endl;
    // m_SetDeployerVoltage(voltage);

    if (m_AtGoal(m_middleGoal, m_getEncoderRadians().value(), vel))
    {
      m_deployerState = MIDDLE;
    }

    break;
  }
  default:
    m_deployerMotor.SetVoltage(units::volt_t{0});
  }
}

void MotorIntake::m_RollerStateMachine()
{

  if (m_showDebug)
  {
    frc::SmartDashboard::PutNumber("Cone Intake current", m_rollerMotor.GetSupplyCurrent());
  }

  if (m_rollerMotor.GetSupplyCurrent() >= m_rollerStallCurrent)
  {
    m_rollerState = MAINTAIN;
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
  case MAINTAIN:
    m_rollerMotor.SetVoltage(units::volt_t{0});
    break;
  default:
    m_rollerMotor.SetVoltage(units::volt_t(0));
  }
}

units::radian_t MotorIntake::m_getEncoderRadians()
{
  // double pos = m_deployerMotor.GetSelectedSensorPosition();
  double pos = m_deployerMotorEncoder.GetPosition();
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
  case MAINTAIN:
    stateStr = "Maintain";
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
 * AtGoal() for ProfiledPIDController does not work so I use this instead
 */
bool MotorIntake::m_AtGoal(double goal, double ang, double vel)
{
  return m_pid.GetSetpoint() == m_pid.GetGoal() && (std::abs(goal - ang)) < m_posErrTolerance && std::abs(vel) < m_velErrTolerance;
}

/**
 * SetVoltage() but with checks with encoder before setting voltage to deployer motor
 */
void MotorIntake::m_SetDeployerVoltage(double volt)
{
  // if (m_getEncoderRadians().value() > MotorIntakeConstants::MOTOR_STOP_BOTTOM || m_getEncoderRadians().value() < MotorIntakeConstants::MOTOR_STOP_TOP)
  // {
  //   m_deployerMotor.SetVoltage(units::volt_t{0});
  //   return;
  // }

  if (m_showDebug)
  {
    frc::SmartDashboard::PutNumber("Cone Intake curpos", m_getEncoderRadians().value());
    frc::SmartDashboard::PutNumber("Cone Intake V to motor", volt);
  }

  m_deployerMotor.SetVoltage(units::volt_t{volt});
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
