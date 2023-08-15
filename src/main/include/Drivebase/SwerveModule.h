#pragma once

#include <iostream>
#include <math.h>
#include <string.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Helpers/TrajectoryCalc.h"
#include "Helpers/Point.h"

#include "SwervePose.h"

//#include <frc/MotorSafety.h>
//#include <frc/smartdashboard/SmartDashboard.h>
//#include <units/units.h>

class SwerveModule
{
    public:
        SwerveModule(int turnID, int driveID, int cancoderID, double offset);

        void periodic();
        void move(Poses::ModulePose target, bool inVolts);

        double calcAngPID(double setAngle);
        double calcDrivePID(double driveSpeed);
        double findError(double setAngle, double angle);
        
        Vector getVelocity();

        // void setP(double p){ akP_ = p; }
        // void setD(double d){ akD_ = d; }

    private:
        WPI_TalonFX turnMotor_;
        WPI_TalonFX driveMotor_;
        WPI_CANCoder cancoder_;

        Poses::ModulePose currPose_;

        TrajectoryCalc trajectoryCalc_;
        bool initTrajectory_;
        double posOffset_, setTrajectoryPos_;
        
        std::string id_;
        double offset_;
        int direction_ = 1;

        double prevTime_, dT_;
        frc::Timer timer_;

        double aPrevError_, aIntegralError_, dPrevError_, dIntegralError_;
};