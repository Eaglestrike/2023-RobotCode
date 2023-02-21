#include "PneumaticsIntake.h"

/**
 * Constructor
 * 
 * @param lInverted Set this to true if setting the left solenoid to a "true" state closes it rather than opens it
 * @param rInverted Set this to true if setting the right solenoid to a "true" state closes it rather than opens it
*/
PneumaticsIntake::PneumaticsIntake(bool lInverted, bool rInverted){
  m_lInverted = lInverted;
  m_rInverted = rInverted;
}

void PneumaticsIntake::Deploy() {
  m_state = DEPLOYED;
}

void PneumaticsIntake::Stow() {
  m_state = STOWED;
}

PneumaticsIntake::State PneumaticsIntake::getState() {
  return m_state;
}

void PneumaticsIntake::Periodic() {
  switch (m_state) {
    case STOWED:
      m_lSolenoid.Set(m_lInverted);
      m_rSolenoid.Set(m_rInverted);
      m_roller.SetVoltage(units::volt_t{0});

      break;
    case DEPLOYED:
      m_lSolenoid.Set(!m_lInverted);
      m_rSolenoid.Set(!m_rInverted);
      m_roller.SetVoltage(units::volt_t{PneumaticsIntakeConstants::ROLLER_MAX_VOLTAGE});

      break;
  }
}