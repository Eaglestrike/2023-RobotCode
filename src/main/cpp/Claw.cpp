#include "Claw.h"

Claw::Claw() : clawPneumatic_(frc::PneumaticsModuleType::CTREPCM, ClawConstants::PNEUMATIC_ID), wheelMotor_(ClawConstants::WHEEL_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless)
{
    open_ = false;
    wheelState_ = IDLE;
    wheelMotor_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Claw::periodic()
{
    frc::SmartDashboard::PutBoolean("Claw Open", open_);

    if (open_)
    {
        clawPneumatic_.Set(false);
    }
    else
    {
        clawPneumatic_.Set(true);
    }

    switch (wheelState_)
    {
    case INTAKING:
    {
        wheelMotor_.Set(ClawConstants::INTAKING_SPEED);
        break;
    }
    case OUTAKING:
    {
        wheelMotor_.Set(ClawConstants::OUTAKING_SPEED);
        break;
    }
    case IDLE:
    {
        wheelMotor_.Set(0);
        break;
    }
    }
}

bool Claw::isOpen()
{
    return open_;
}
Claw::WheelState Claw::getWheelState()
{
    return wheelState_;
}

void Claw::setOpen(bool open)
{
    open_ = open;
}
void Claw::setWheelState(WheelState wheelState)
{
    wheelState_ = wheelState;
}