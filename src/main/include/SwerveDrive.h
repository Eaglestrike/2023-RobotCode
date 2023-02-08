#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "SwervePose.h"
#include "SwervePath.h"

#include "SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>


class SwerveDrive
{
    public:
        SwerveDrive();
        void setYaw(double yaw);
        
        void periodic(double yaw, Controls* controls);
        void drive(double xSpeed, double ySpeed, double turn);
        //void driveAutoPose(double yaw, SwervePose pose);
        void drivePose(SwervePose pose);
        void adjustPos(SwervePose pose);

        void calcModules(double xSpeed, double ySpeed, double xAcc, double yAcc, double turn, double turnAacc, bool inVolts);

        void calcOdometry();
        void reset();

        double getX();
        double getY();
        pair<double, double> getXYVel();
        double getYaw();
        void setPos(pair<double, double> xy);

        void updateAprilTagFieldXY();
        int checkScoringPos();
        
    private:
        SwerveModule* topRight_ = new SwerveModule(SwerveConstants::TR_TURN_ID, SwerveConstants::TR_DRIVE_ID, SwerveConstants::TR_CANCODER_ID, SwerveConstants::TR_CANCODER_OFFSET);
        SwerveModule* topLeft_ = new SwerveModule(SwerveConstants::TL_TURN_ID, SwerveConstants::TL_DRIVE_ID, SwerveConstants::TL_CANCODER_ID, SwerveConstants::TL_CANCODER_OFFSET);
        SwerveModule* bottomRight_ = new SwerveModule(SwerveConstants::BR_TURN_ID, SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_CANCODER_ID, SwerveConstants::BR_CANCODER_OFFSET);
        SwerveModule* bottomLeft_ = new SwerveModule(SwerveConstants::BL_TURN_ID, SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_CANCODER_ID, SwerveConstants::BL_CANCODER_OFFSET);

        SwervePath tagPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};

        double robotX_, robotY_, yaw_;
        TrajectoryCalc xTagTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yTagTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yawTagTraj_{SwerveConstants::MAX_AV * 0.1, SwerveConstants::MAX_AA * 0.1, 0, 0, 0, 0};
        //double aprilTagX_, aprilTagY_;

        double prevTime_, dT_;

        frc::Timer timer_;
        double tagFollowingStartTime_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_;

        bool trackingTag_, foundTag_;
        int tagPos_, prevTag_;

};