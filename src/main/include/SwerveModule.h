#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string.h>
#include "Controls.h"
#include "Constants.h"
#include <units/length.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/MathUtil.h>

#define M_PI 3.14159265358979323846 //for simulation

class SwerveModule
{
    public:
        SwerveModule(int turnID, int driveID, int cancoderID, double offset, bool inverted);

        double calcAngPID(double setAngle);
        double calcDrivePID(double driveSpeed);
        double findError(double setAngle, double angle);

        frc::SwerveModuleState getState();
        frc::SwerveModuleState getOptState(frc::SwerveModuleState state);
        
        double getVelocity();
        double getYaw();

        void setAngMotorVoltage(double voltage);
        void setSpeedMotor(double power);
        void setSpeedMotorVolts(units::volt_t volts);

        void setP(double p){ akP_ = p; }
        void setD(double d){ akD_ = d; }

    private:

        WPI_TalonFX angleMotor_;
        WPI_TalonFX speedMotor_;
        WPI_CANCoder canCoder_;

        double maxV = 1440;
        double maxA = 14400 * 10;
        double kP = 0.05;
        double kD = 0;
        double kV = 1 / 261.864;
        double kVI = -131.727;
        double kA = 0;

        units::meters_per_second_t talonVelToMps(double vel);
        
        std::string id_;
        double offset_;
        int direction_ = 1;

        double prevTime_, dT_;
        frc::Timer timer_;

        double aPrevError_, aIntegralError_, dPrevError_, dIntegralError_;

        double akP_ = 0.08; //TODO tune values
        double akI_ = 0.0;
        double akD_ = 0.001;

        double dkP_ = 0.0; //TODO tune values
        double dkI_ = 0.0;
        double dkD_ = 0.0;

};