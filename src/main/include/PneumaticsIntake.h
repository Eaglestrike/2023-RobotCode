/**
 * Pneumatics-deployed intake
*/

#ifndef PNEUMATICS_INTAKE_H
#define PNEUMATICS_INTAKE_H

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#include <ctre/Phoenix.h>
#include <frc/fmt/Units.h>
#include <frc/Solenoid.h>

#include "Constants.h"

/**
 * Intake that is deployed by a motor (rather than pneumatics)
 * 
 * Used with the cube intake
*/
class PneumaticsIntake {
public:
  enum State {
    STOWED,
    DEPLOYED
  };  

  PneumaticsIntake(bool = false, bool = false);

  void Periodic();

  void Deploy();
  void Stow();

  State getState();
private:
  frc::Solenoid m_lSolenoid{
    PneumaticsIntakeConstants::USING_CTRE ? frc::PneumaticsModuleType::CTREPCM : frc::PneumaticsModuleType::REVPH,
    PneumaticsIntakeConstants::LEFT_SOLENOID_ID};
  frc::Solenoid m_rSolenoid{
    PneumaticsIntakeConstants::USING_CTRE ? frc::PneumaticsModuleType::CTREPCM : frc::PneumaticsModuleType::REVPH,
    PneumaticsIntakeConstants::RIGHT_SOLENOID_ID};

  WPI_TalonFX m_roller{MotorIntakeConstants::ROLLER_MOTOR_ID};  // for spinning the rollers

  State m_state{STOWED};

  bool m_lInverted;
  bool m_rInverted;
};

#endif