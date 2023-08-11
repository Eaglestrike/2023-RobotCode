#pragma once

#include <iostream>
#include <vector>
#include <utility>
#include <map>

#include <ctre/Phoenix.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>

#include "Helpers/Point.h"

#include "SwerveConstants.h"
#include "SwervePose.h"
#include "SwervePath.h"
#include "SwerveModule.h"

class SwerveDrive
{
    public:
        struct Config{
            bool isBlue; //What side the robot is
            bool isSlow = false;
            bool isPanic = false;
        };
        SwerveDrive();
        void setYaw(double yaw);
        
        void periodic(double yaw, double tilt, std::vector<double> data);
        void trim(double xLineupTrim, double yLineupTrim_);
        void inch(double inchUp, double inchDown, double inchLeft, double inchRight, double isInch);
        void teleopPeriodic(bool score, bool forward, int scoringLevel, bool islockWheels, bool autoBalance);

        void setTarget(double xStrafe, double yStrafe, double rotation);
        void setPanic(bool panic);

        void manualScore(int scoringLevel);

        void drive(Vector strafe, double turn);
        void lockWheels();
        void drivePose(const Poses::SwervePose pose);
        void adjustPos(const Poses::SwervePose pose);

        void calcModules(double xSpeed, double ySpeed, /*double xAcc, double yAcc,*/ double turn, /*double turnAacc,*/ bool inVolts);

        void calcOdometry();
        void reset();

        double getX();
        double getY();
        std::pair<double, double> getXYVel();
        double getYaw();
        void setPos(std::pair<double, double> xy);

        void updateAprilTagFieldXY(double tilt, std::vector<double> data);
        std::pair<double, double> checkScoringPos(int scoringLevel);
        void setScoringPos(int scoringPos);
        int getScoringPos();

        // void resetYawTagOffset();
        // double getYawTagOffset();
        
    private:
        Vector strafe_; //Field oriented
        double rotation_;

        Config config_;

        SwerveModule* topRight_ = new SwerveModule(SwerveConstants::TR_TURN_ID, SwerveConstants::TR_DRIVE_ID, SwerveConstants::TR_CANCODER_ID, SwerveConstants::TR_CANCODER_OFFSET);
        SwerveModule* topLeft_ = new SwerveModule(SwerveConstants::TL_TURN_ID, SwerveConstants::TL_DRIVE_ID, SwerveConstants::TL_CANCODER_ID, SwerveConstants::TL_CANCODER_OFFSET);
        SwerveModule* bottomRight_ = new SwerveModule(SwerveConstants::BR_TURN_ID, SwerveConstants::BR_DRIVE_ID, SwerveConstants::BR_CANCODER_ID, SwerveConstants::BR_CANCODER_OFFSET);
        SwerveModule* bottomLeft_ = new SwerveModule(SwerveConstants::BL_TURN_ID, SwerveConstants::BL_DRIVE_ID, SwerveConstants::BL_CANCODER_ID, SwerveConstants::BL_CANCODER_OFFSET);

        SwervePath tagPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};

        double robotX_, robotY_, yaw_/*, yawTagOffset_*/;
        TrajectoryCalc xTagTraj_{{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0}};
        TrajectoryCalc yTagTraj_{{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0}};
        TrajectoryCalc yawTagTraj_{{SwerveConstants::MAX_AV * 0.7, SwerveConstants::MAX_AA * 0.7, 0, 0, 0, 0}};
        //double aprilTagX_, aprilTagY_;

        double prevTime_, dT_;

        frc::Timer timer_;
        double tagFollowingStartTime_;

        double trSpeed_, brSpeed_, tlSpeed_, blSpeed_, trAngle_, brAngle_, tlAngle_, blAngle_, holdingYaw_;

        bool trackingTag_, trackingPlayerStation_, foundTag_, isHoldingYaw_/*, inching_*/;
        int setTagPos_, prevTag_, prevUniqueVal_, numLargeDiffs_;
        double xLineupTrim_, yLineupTrim_;

        std::map<double, std::pair<std::pair<double, double>, std::pair<double, double>>> prevPoses_;

};