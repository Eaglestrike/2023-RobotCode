#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <map>

#include "Controls/Controller.h"

#include "SwerveConstants.h"
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
        
        void periodic(double yaw, double tilt, vector<double> data);
        void teleopPeriodic(bool forward, bool panic, int scoringLevel);
        void drive(double xSpeed, double ySpeed, double turn);
        void lockWheels();
        void drivePose(SwervePose pose);
        void adjustPos(SwervePose pose);

        void calcModules(double xSpeed, double ySpeed, /*double xAcc, double yAcc,*/ double turn, /*double turnAacc,*/ bool inVolts);

        void calcOdometry();
        void reset();

        double getX();
        double getY();
        pair<double, double> getXYVel();
        double getYaw();
        void setPos(pair<double, double> xy);

        void updateAprilTagFieldXY(double tilt, vector<double> data);
        pair<double, double> checkScoringPos(int scoringLevel);
        void setScoringPos(int scoringPos);
        int getScoringPos();

        // void resetYawTagOffset();
        // double getYawTagOffset();
        
    private:
        Controller controls_;

        SwerveModule* topRight_ = new SwerveModule(SwerveConstants::TR_TURN_ID, SwerveConstants::TR_DRIVE_ID, SwerveConstants::TR_CANCODER_ID, SwerveConstants::TR_CANCODER_OFFSET);
        SwerveModule* topLeft_ = new SwerveModule(SwerveConstants::TL_TURN_ID, SwerveConstants::TL_DRIVE_ID, SwerveConstants::TL_CANCODER_ID, SwerveConstants::TL_CANCODER_OFFSET);
        SwerveModule* bottomRight_ = new SwerveModule(SwerveConstants::BR_TURN_ID, SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_CANCODER_ID, SwerveConstants::BR_CANCODER_OFFSET);
        SwerveModule* bottomLeft_ = new SwerveModule(SwerveConstants::BL_TURN_ID, SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_CANCODER_ID, SwerveConstants::BL_CANCODER_OFFSET);

        SwervePath tagPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};

        double robotX_, robotY_, yaw_/*, yawTagOffset_*/;
        TrajectoryCalc xTagTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yTagTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yawTagTraj_{SwerveConstants::MAX_AV * 0.7, SwerveConstants::MAX_AA * 0.7, 0, 0, 0, 0};
        //double aprilTagX_, aprilTagY_;

        double prevTime_, dT_;

        frc::Timer timer_;
        double tagFollowingStartTime_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_, holdingYaw_;

        bool trackingTag_, trackingPlayerStation_, foundTag_, isHoldingYaw_/*, inching_*/;
        int setTagPos_, prevTag_, prevUniqueVal_, numLargeDiffs_;
        double xLineupTrim_, yLineupTrim_;

        map<double, pair<pair<double, double>, pair<double, double>>> prevPoses_;

};