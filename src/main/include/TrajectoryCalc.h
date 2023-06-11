#pragma once
#include <math.h>
#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class TrajectoryCalc
{
    public:
        TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA);
        TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA, double kVI);

        void setKP(double kP);
        void setKD(double kD);
        void setKV(double kV);
        void setKA(double kA);
        void setKVI(double kVI);

        void generateTrajectory(double pos, double setPos, double vel);
        //void generateVelTrajectory(double setVel, double vel);

        tuple<double, double, double> getProfile();
        //pair<double, double> getVelProfile();

        double calcPower(double pos, double vel);
        //double calcVelPower(double vel);

        void setPrintError(bool printError);

    private:
        const double MAX_V, MAX_A;
        double kP_, kD_, kV_, kA_, kVI_;

        double /*prevAbsoluteError_,*/ setPos_, setVel_, initPos_, initVel_, startTime_;

        int direction_;
        double cruiseSpeed_, cruiseDist_, accelDist_, deccelDist_, cruiseTime_, accelTime_, deccelTime_;

        bool printError_ = false;

        frc::Timer timer_;
};