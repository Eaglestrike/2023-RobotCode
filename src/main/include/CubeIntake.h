// /**
//  * Cube Intake
// */

// #ifndef CUBE_INTAKE_H
// #define CUBE_INTAKE_H

// #define _USE_MATH_DEFINES
// #include <algorithm>
// #include <cmath>

// #include <ctre/Phoenix.h>
// #include <frc/controller/ProfiledPIDController.h>
// #include <frc/fmt/Units.h>
// #include <frc/Timer.h>
// #include <frc/trajectory/TrapezoidProfile.h>
// #include <units/angle.h>
// #include <units/angular_acceleration.h>
// #include <units/angular_velocity.h>
// #include <units/time.h>

// #include "Constants.h"
// #include "Helpers.h"

// class CubeIntake {
// public:
//   // state machine
//   enum State {
//     STOWED,    // if fully in
//     DEPLOYED,  // if fully out
//     STOWING,   // if currently stopping
//     DEPLOYING, // if currently deploying
//   };

//   CubeIntake();

//   void RobotInit();
//   void Periodic();

//   void Deploy();
//   void Stow();

//   State getState();

//   void Reset();
//   void ResetEncoderPosition();
//   void ResetPID();
//   void ResetAcceleration();
// private:
//   State m_state{STOWED};

//   WPI_TalonFX m_deployer{CubeIntakeConstants::DEPLOYER_MOTOR_ID}; // for deploying the intake
//   WPI_TalonFX m_roller{CubeIntakeConstants::ROLLER_MOTOR_ID};  // for spinning the rollers

//   frc::ProfiledPIDController<units::radians> m_pid{
//     CubeIntakeConstants::kP,
//     CubeIntakeConstants::kI,
//     CubeIntakeConstants::kD,
//     frc::TrapezoidProfile<units::radians>::Constraints{
//       units::radians_per_second_t{CubeIntakeConstants::MAX_VELOCITY},
//       units::radians_per_second_squared_t{CubeIntakeConstants::MAX_ACCELERATION}
//     }
//   };
//   units::radians_per_second_t m_lastSpeed{units::radians_per_second_t{0}};
//   units::second_t m_lastTime{frc::Timer::GetFPGATimestamp()};

//   units::radian_t m_getEncoderRadians();
// };

// #endif