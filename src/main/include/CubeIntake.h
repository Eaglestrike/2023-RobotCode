/**
 * Cube Intake
*/

#ifndef CUBE_INTAKE_H
#define CUBE_INTAKE_H

#include <ctre/Phoenix.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/fmt/Units.h>
#include <frc/Timer.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "Constants.h"

class CubeIntake {
public:
  CubeIntake() = default;

  void RobotInit();
  void AutonomousPeriodic();
  void TeleopPeriodic();

  void Deploy();
  void Stow();

  bool isDeployed();
  bool isStowed();

  void Reset();
  void ResetEncoderPosition();
  void ResetPID();
private:
  // state machine
  enum State {
    STOWED,    // if fully in
    DEPLOYED,  // if fully out
    STOWING,   // if currently stopping
    DEPLOYING, // if currently deploying
  };

  State m_state{STOWED};

  WPI_TalonFX m_deployer{CubeIntakeConstants::DEPLOYER_MOTOR_ID}; // for deploying the intake
  WPI_TalonFX m_roller{CubeIntakeConstants::ROLLER_MOTOR_ID};  // for spinning the rollers

  frc::ProfiledPIDController<units::radian> m_pid{
    CubeIntakeConstants::kP,
    CubeIntakeConstants::kI,
    CubeIntakeConstants::kD,
    frc::TrapezoidProfile<units::radian>::Constraints{
      units::angle::radian_t{CubeIntakeConstants::MAX_VELOCITY},
      units::angle::radian_t{CubeIntakeConstants::MAX_ACCELERATION}
    }
  };

  // below code gives errors
  // units::radians_per_second_t m_lastSpeed = units::radians_per_second_t{0};
  // units::second_t m_lastTime = frc2::Timer::GetFPGATimestamp();
};

#endif