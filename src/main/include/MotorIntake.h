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
class MotorIntake {
public:
  // state machine
  enum State {
    STOWED,    // if fully in
    DEPLOYED,  // if fully out
    STOWING,   // if currently stopping
    DEPLOYING, // if currently deploying
  };

  MotorIntake();

  void RobotInit();
  void Periodic();

  void Deploy();
  void Stow();

  State getState();

  void Reset();
  void ResetEncoderPosition();
  void ResetPID();
  void ResetAcceleration();
private:
  State m_state{STOWED};

  WPI_TalonFX m_deployer{MotorIntakeConstants::DEPLOYER_MOTOR_ID}; // for deploying the intake
  WPI_TalonFX m_roller{MotorIntakeConstants::ROLLER_MOTOR_ID};  // for spinning the rollers

  frc::ProfiledPIDController<units::radians> m_pid{
    MotorIntakeConstants::kP,
    MotorIntakeConstants::kI,
    MotorIntakeConstants::kD,
    frc::TrapezoidProfile<units::radians>::Constraints{
      units::radians_per_second_t{MotorIntakeConstants::MAX_VELOCITY},
      units::radians_per_second_squared_t{MotorIntakeConstants::MAX_ACCELERATION}
    }
  };
  units::radians_per_second_t m_lastSpeed{units::radians_per_second_t{0}};
  units::second_t m_lastTime{frc::Timer::GetFPGATimestamp()};

  units::radian_t m_getEncoderRadians();
};

#endif