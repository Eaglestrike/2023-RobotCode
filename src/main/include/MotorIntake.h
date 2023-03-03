/**
 * Motor-deployed intake, opposed to pneumatics-deployed
 */

#ifndef MOTOR_INTAKE_H
#define MOTOR_INTAKE_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <string>

#include <ctre/Phoenix.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/fmt/Units.h>
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "Constants.h"
#include "Helpers.h"

/**
 * Intake that is deployed by a motor (rather than pneumatics)
 *
 * This should be used with the cone intake.
 *
 * @note The roller is stopped if the deployer is not down, so if smartdashboard says "Intake"
 * or "Outtake" even though it is not spinning, first check if the deployer is not down.
 */
class MotorIntake
{
public:
  // deployer state machine
  enum DeployerState
  {
    STOWED,    // if fully in
    DEPLOYED,  // if fully out
    STOWING,   // if currently stopping
    DEPLOYING, // if currently deploying
  };

  // roller state machine
  enum RollerState
  {
    INTAKE,  // roller spins such that it's intaking
    STOP,    // roller does not spin
    OUTTAKE, // roller spins such that it's outtaking
  };

  // overall state machine
  enum ConeIntakeState
  {
    IDLE,       // stored in the robot with no cone inside
    IDLING,     // currently stowing the cone intake without a cone
    CONSUMING,  // currently intaking the cone and stowing it
    SPITTING,   // currently lowering the intake (with cone inside)
    CONSUMED,   // cone is in the robot and stored
    INTAKE_DOWN // the intake is down and the rollers are spinning to spit out the cone
  };

  MotorIntake(bool = true);

  void RobotInit();
  void Periodic();

  void Consume();
  void Spit();
  void Idle();

  bool IsStored();
  bool IntakeDown();
  bool IsIdle();

  DeployerState getDeployerState();
  RollerState getRollerState();

  void Deploy();
  void Stow();

  void Reset();
  void ResetEncoderPosition();
  void ResetPID();
  void ResetAcceleration();

private:
  // state machines
  DeployerState m_deployerState{STOWED};
  RollerState m_rollerState{INTAKE};
  ConeIntakeState m_intakeState{IDLE};

  // motors
  rev::CANSparkMax m_deployerMotor{MotorIntakeConstants::DEPLOYER_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless}; // for deploying the intake
  WPI_TalonFX m_rollerMotor{MotorIntakeConstants::ROLLER_MOTOR_ID};                                                   // for spinning the rollers

  // motion profiled pid controller
  frc::ProfiledPIDController<units::radians> m_pid{
      MotorIntakeConstants::kP,
      MotorIntakeConstants::kI,
      MotorIntakeConstants::kD,
      frc::TrapezoidProfile<units::radians>::Constraints{
          units::radians_per_second_t{MotorIntakeConstants::MAX_VELOCITY},
          units::radians_per_second_squared_t{MotorIntakeConstants::MAX_ACCELERATION}}};

  // feedforward stuff; currently unused
  units::radians_per_second_t m_lastSpeed{units::radians_per_second_t{0}};
  units::second_t m_lastTime{frc::Timer::GetFPGATimestamp()};

  // if info should be shown on smartdashboard or not
  bool m_showSmartDashboard;

  // util methods
  units::radian_t m_getEncoderRadians();

  bool m_MotorsNotNeeded();

  void m_DeployerStateMachine();
  void m_RollerStateMachine();

  void m_PutCurrentDeployerState();
  void m_PutCurrentRollerState();
  void m_PutCurrentConeIntakeState();
};

#endif