#include "Intake/PneumaticsIntake.h"

/**
 * Constructor
 *
 * @param lInverted Set this to true if setting the left solenoid to a "true" state closes it rather than opens it
 * @param rInverted Set this to true if setting the right solenoid to a "true" state closes it rather than opens it
 */
PneumaticsIntake::PneumaticsIntake(bool lInverted, bool rInverted)
{
  m_lInverted = lInverted;
  m_rInverted = rInverted;
}

/**
 * Deploys the intake
 *
 * @param mode If set to INTAKE, roller instakes the cube. If set to STOP, roller does not move. If set to OUTTAKE, roller outtakes the cube.
 */
void PneumaticsIntake::Deploy()
{
  m_state = DEPLOYED;
}

void PneumaticsIntake::Stow()
{
  m_state = STOWED;
}

PneumaticsIntake::State PneumaticsIntake::getState()
{
  return m_state;
}

PneumaticsIntake::RollerMode PneumaticsIntake::getRollerMode()
{
  return m_mode;
}

/**
 * Set the roller mode
 *
 * The roller mode determines if the roller intakes, outtakes, or does not roll
 * when the intake is deployed.
 */
void PneumaticsIntake::setRollerMode(PneumaticsIntake::RollerMode mode)
{
  m_mode = mode;
}

void PneumaticsIntake::Periodic()
{
  switch (m_state)
  {
  case STOWED:
    m_lSolenoid.Set(m_lInverted);
    m_rSolenoid.Set(m_rInverted);
    m_roller.SetVoltage(units::volt_t{0});

    break;
  case DEPLOYED:
    m_lSolenoid.Set(!m_lInverted);
    m_rSolenoid.Set(!m_rInverted);
    if (m_mode == RollerMode::INTAKE)
    {
      m_roller.SetVoltage(units::volt_t{PneumaticsIntakeConstants::ROLLER_INTAKE_VOLTAGE});
    }
    else if (m_mode == RollerMode::STOP)
    {
      m_roller.SetVoltage(units::volt_t{0});
    }
    else
    {
      m_roller.SetVoltage(units::volt_t{PneumaticsIntakeConstants::ROLLER_OUTTAKE_VOLTAGE});
    }

    break;
  }
}