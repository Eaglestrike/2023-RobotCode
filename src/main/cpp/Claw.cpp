#include "Claw.h"

Claw::Claw() : clawPneumatic_(frc::PneumaticsModuleType::CTREPCM, ClawConstants::PNEUMATIC_ID), wheelMotor_(ClawConstants::WHEEL_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless)
{
    open_ = false;
    wheelSpeed_ = 0;
    wheelMotor_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Claw::periodic()
{
    frc::SmartDashboard::PutBoolean("Claw Open", open_);

    clawPneumatic_.Set(open_);

    if(wheelSpeed_ == ClawConstants::INTAKING_SPEED && wheelMotor_.GetOutputCurrent() > ClawConstants::RETAINING_CURRENT)
    {
        wheelMotor_.SetVoltage(units::volt_t{ClawConstants::RETAINING_SPEED});
    }
    else
    {
        wheelMotor_.SetVoltage(units::volt_t{wheelSpeed_});
    }
    
}

bool Claw::isOpen()
{
    return open_;
}

double Claw::wheelSpeed()
{
    return wheelSpeed_;
}

void Claw::setOpen(bool open)
{
    open_ = open;
}

void Claw::setWheelSpeed(double speed)
{
    wheelSpeed_ = speed;
}