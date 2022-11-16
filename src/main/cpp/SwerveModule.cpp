#include "SwerveModule.h"

/**
 * Iniitialize a swerve module. Mostly setting ports, also if the encoder has an offset/motor is inverted
**/
SwerveModule::SwerveModule(int angMotorPort, int speedMotorPort, int canCoderPort, double offset, bool inverted) : angleMotor_{angMotorPort, "Drivebase"},
speedMotor_{speedMotorPort, "Drivebase"}, canCoder_{canCoderPort, "Drivebase"}, offset_{offset}
{
    speedMotor_.SetInverted(inverted);

    angleMotor_.SetSelectedSensorPosition(0);
    speedMotor_.SetSelectedSensorPosition(0);

    angleMotor_.SetNeutralMode(NeutralMode::Brake);
    speedMotor_.SetNeutralMode(NeutralMode::Brake);
}

/**
 * Converts the raw encoder speed of the driving swerve motor to its speed in meters per second
 * @returns velocity in mps
**/
units::meters_per_second_t SwerveModule::talonVelToMps(double vel)
{
    double wheel_radius = 0.05;                      // in meters
    double meters_per_rev = wheel_radius * 2 * M_PI; // wheel circumberence
    double ticks_per_rev = 12650;
    return units::meters_per_second_t{vel / 0.1 * (meters_per_rev / ticks_per_rev)};
}

/**
 * Gets the drive speed and angle of a swerve module
 * @returns a SwerveModuleState encapsulating that info
**/
frc::SwerveModuleState SwerveModule::getState()
{
    frc::SwerveModuleState state;
    state.speed = talonVelToMps(speedMotor_.GetSelectedSensorVelocity());
    //converts module's angle (normalized to [-180, 180]) to a Rotation2d object
    state.angle = frc::Rotation2d{units::angle::degree_t{frc::InputModulus(angleMotor_.GetSelectedSensorVelocity() + offset_, -180.0, 180.0)}};
    return state;
}

/**
 * @returns the "optimized" version of the @param state 
**/
frc::SwerveModuleState SwerveModule::getOptState(frc::SwerveModuleState state)
{
    double yaw = frc::InputModulus(canCoder_.GetAbsolutePosition() + offset_, -180.0, 180.0);
    frc::SwerveModuleState opt_state = frc::SwerveModuleState::Optimize(state, units::degree_t(yaw));
    return opt_state;
}


void SwerveModule::setAngMotorVoltage(double volts)
{
    angleMotor_.SetVoltage(units::volt_t{volts});
}

void SwerveModule::setSpeedMotor(double power)
{
    speedMotor_.Set(ControlMode::PercentOutput, power);
}

void SwerveModule::setSpeedMotorVolts(units::volt_t volts) {
    speedMotor_.SetVoltage(volts);
}

//note: in raw ticks, not mps
double SwerveModule::getVelocity()
{
    return speedMotor_.GetSelectedSensorVelocity();
}

//in degrees
double SwerveModule::getYaw()
{
    return canCoder_.GetAbsolutePosition() + offset_;
}