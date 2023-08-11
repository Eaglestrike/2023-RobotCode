#pragma once
#include <math.h>
#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "GeneralConstants.h"

#include "Helpers/GeneralPoses.h"

// Gains = Constants -> converting factors to give voltage

/// @brief a feedforward-PD class using trapezoidal motion with a feedback PD loop
class TrajectoryCalc
{
    public:
        struct TrajCalcParam{
            double maxV; //Max Velocity
            double maxA; //Max Acceleration
            double kP; //P in PD controller, controls strength of adjustment due to displacement
            double kD; //D in PD controller, controls strength of mitigation of overshoot
            double kV; //Multiplied to target velocity to convert velocity -> volts
            double kA; //Multiplied to target acceleration to convert acceleration -> volts
            double kVI = 0.0; //Offsets target velocity (subtracted), to compensate the acceleration given by kV (unsure)
        };
        TrajectoryCalc(TrajCalcParam param);

        void setKP(double kP);
        void setKD(double kD);
        void setKV(double kV);
        void setKA(double kA);
        void setKVI(double kVI);

        void generateTrajectory(double pos, double setPos, double vel);
        //void generateVelTrajectory(double setVel, double vel);

        Poses::Pose1D getProfile();
        //pair<double, double> getVelProfile();

        double calcPower(double pos, double vel);
        //double calcVelPower(double vel);

        void setPrintError(bool printError);

    private:
        TrajCalcParam parameters_;

        double /*prevAbsoluteError_,*/ setPos_, setVel_, initPos_, initVel_, startTime_;

        int direction_;
        double cruiseSpeed_, cruiseDist_, accelDist_, deccelDist_, cruiseTime_, accelTime_, deccelTime_;

        bool printError_ = false;

        frc::Timer timer_;
};