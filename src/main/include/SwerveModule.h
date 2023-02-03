#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string.h>
#include "Controls.h"
#include "Constants.h"
#include "Helpers.h"
#include "TrajectoryCalc.h"

//#include <frc/MotorSafety.h>
//#include <frc/smartdashboard/SmartDashboard.h>
//#include <units/units.h>

class SwerveModule
{
    public:
        SwerveModule(int turnID, int driveID, int cancoderID, double offset);

        void periodic(double driveSpeed, double angle, bool inVolts);
        void move(double driveSpeed, double angle, bool inVolts);

        double calcAngPID(double setAngle);
        double calcDrivePID(double driveSpeed);
        double findError(double setAngle, double angle);
        
        double getDriveVelocity();
        double getAngle();

        void setP(double p){ akP_ = p; }
        void setD(double d){ akD_ = d; }

    private:
        WPI_TalonFX turnMotor_;
        WPI_TalonFX driveMotor_;
        WPI_CANCoder cancoder_;

        double maxV = 1440;
        double maxA = 14400 * 10;
        double kP = 0.05;
        double kD = 0;
        double kV = 1 / 261.864;
        double kVI = -131.727;
        double kA = 0;
        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;
        double posOffset_, setTrajectoryPos_;
        
        string id_;
        double offset_;
        int direction_ = 1;

        double prevTime_, dT_;
        frc::Timer timer_;

        double aPrevError_, aIntegralError_, dPrevError_, dIntegralError_;

        double akP_ = 0.08; //COULDO tune values
        double akI_ = 0.0;
        double akD_ = 0.001;

        double dkP_ = 0.0;
        double dkI_ = 0.0;
        double dkD_ = 0.0;

};