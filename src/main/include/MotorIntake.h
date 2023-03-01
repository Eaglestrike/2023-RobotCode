/**
 * Motor-deployed intake, opposed to pneumatics-deployed
 */

#ifndef MOTOR_INTAKE_H
#define MOTOR_INTAKE_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#include <ctre/Phoenix.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/fmt/Units.h>
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "Constants.h"
#include "Helpers.h"

/**
 * Intake that is deployed by a motor (rather than pneumatics)
 *
 * Potentially used with the cone intake
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

  MotorIntake();

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
  DeployerState m_deployerState{STOWED};
  RollerState m_rollerState{INTAKE};
  ConeIntakeState m_intakeState{IDLE};

  WPI_TalonFX m_deployerMotor{MotorIntakeConstants::DEPLOYER_MOTOR_ID}; // for deploying the intake
  WPI_TalonFX m_rollerMotor{MotorIntakeConstants::ROLLER_MOTOR_ID};     // for spinning the rollers

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

  units::radian_t m_getEncoderRadians();

  void m_DeployerStateMachine();
  void m_RollerStateMachine();

  bool m_MotorsNotNeeded();
};

#endif