#pragma once

namespace MotorIntakeConstants
{
    const int DEPLOYER_MOTOR_ID = 0; // TODO get value
    const int ROLLER_MOTOR_ID = 0;   // TODO get value

    const double ENCODER_DEPLOYED_TARGET = 512;

    const double DEPLOYER_MAX_VOLTAGE = 0; // TODO measure capped voltage for deployer
    const double ROLLER_MAX_VOLTAGE = 0;   // TODO measure voltage for cube/cone to actually pass through

    const double kP = 0; // TODO tune
    const double kI = 0; // TODO tune
    const double kD = 0; // TODO tune

    const double MAX_VELOCITY = M_PI / 2; // TODO tune - radians per second
    const double MAX_ACCELERATION = M_PI; // TODO tune - radians per second squared

    const double POS_ERR_TOLERANCE = 0.01; // TODO tune - error tolerance, in radians
    const double VEL_ERR_TOLERANCE = 0.1;  // TODO tune - error tolerance, in radians/s
} // namespace MotorIntakeConstants


namespace PneumaticsIntakeConstants
{
    const bool USING_CTRE = true; // if this is set to false, then you are using the REV pneumatics hub

    const int LEFT_SOLENOID_ID = 0;
    const int RIGHT_SOLENOID_ID = 7;
    const int ROLLER_MOTOR_ID = 13;

    const double ROLLER_INTAKE_VOLTAGE = -2.5;
    const double ROLLER_OUTTAKE_VOLTAGE = 2.5;
} // namespace PneumaticsIntakeConstants

namespace CubeGrabberConstants {
    const int LEFT_MOTOR_ID = 41;
    const int RIGHT_MOTOR_ID = 43;

    const double CLOCKWISE_VOLTAGE = 3;
    const double COUNTERCLOCKWISE_VOLTAGE = -3; 
}