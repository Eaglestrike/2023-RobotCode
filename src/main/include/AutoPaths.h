#pragma once

#include <frc/Timer.h>

#include "SwerveDrive.h"
#include "SwervePath.h"
#include "TwoJointArm.h"
#include "Constants.h"
#include "TwoJointArmProfiles.h"
#include <vector>

class AutoPaths
{
    public:
        enum Path
        {
            BIG_BOY,
            PRELOADED_CONE,
            PRELOADED_CUBE,
            FIRST_CONE,
            FIRST_CUBE,
            SECOND_CONE,
            SECOND_CUBE,
            AUTO_DOCK,
            NOTHING,
            DRIVE_BACK_DUMB,
            WAIT_5_SECONDS
        };
        AutoPaths(SwerveDrive* swerveDrive, TwoJointArm* arm);
        void setActions(Path a1, Path a2, Path a3, Path a4);
        vector<Path> getActions();

        void startTimer();
        void setActionsSet(bool actionsSet);
        void setPathSet(bool pathSet);

        void periodic();
        double initYaw();
        pair<double, double> initPos();

        int pointNum();

        void setMirrored(bool mirrored);

        bool getClawOpen();
        bool getForward();
        double getWheelSpeed();
        TwoJointArmProfiles::Positions getArmPosition();
        bool cubeIntaking();
        bool coneIntaking();
    private:
        vector<Path> actions_;
        Path path_;
        SwerveDrive* swerveDrive_;
        TwoJointArm* arm_;

        TrajectoryCalc xTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yTraj_{SwerveConstants::MAX_LV * 0.7, SwerveConstants::MAX_LA * 0.7, 0, 0, 0, 0};
        TrajectoryCalc yawTraj_{SwerveConstants::MAX_AV * 0.4, SwerveConstants::MAX_AA * 0.4, 0, 0, 0, 0};

        frc::Timer timer_;
        frc::Timer failsafeTimer_;
        double startTime_, curveSecondStageStartTime_, placingStartTime_;
        bool nextPointReady_, failsafeStarted_, dumbTimerStarted_, pathSet_, pathGenerated_, curveSecondStageGenerated_, yawStageGenerated_, actionsSet_, mirrored_, cubeIntaking_, coneIntaking_, placingTimerStarted_;

        //vector<SwervePath> swervePaths_;
        int actionNum_;
        vector<SwervePose> swervePoints_;
        SwervePath currPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};
        int pointNum_;

        void setPath(Path path);
        Path getPath();

        bool clawOpen_, forward_;
        double wheelSpeed_;
        TwoJointArmProfiles::Positions armPosition_;
};