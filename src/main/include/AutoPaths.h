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
            PRELOADED_CONE_MID,
            PRELOADED_CUBE_MID,
            PRELOADED_CONE_HIGH,
            PRELOADED_CUBE_HIGH,
            FIRST_CONE_MID,
            FIRST_CUBE_HIGH,
            FIRST_CONE_DOCK,
            FIRST_CUBE_DOCK,
            SECOND_CONE,
            SECOND_CUBE_MID,
            SECOND_CUBE_HIGH,
            SECOND_CONE_DOCK,
            SECOND_CUBE_DOCK,
            AUTO_DOCK,
            TAXI_DOCK_DUMB,
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
        void setGyros(double yaw, double pitch, double roll);
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

        TrajectoryCalc xTraj_{SwerveConstants::MAX_LV * 0.9, SwerveConstants::MAX_LA * 0.9, 0, 0, 0, 0};
        TrajectoryCalc yTraj_{SwerveConstants::MAX_LV * 0.9, SwerveConstants::MAX_LA * 0.9, 0, 0, 0, 0};
        TrajectoryCalc yawTraj_{SwerveConstants::MAX_AV * 0.2, SwerveConstants::MAX_AA * 0.2, 0, 0, 0, 0};

        frc::Timer timer_;
        frc::Timer failsafeTimer_;
        double startTime_, curveSecondStageStartTime_, placingStartTime_, yaw_, pitch_, roll_;
        bool nextPointReady_, failsafeStarted_, dumbTimerStarted_, pathSet_, pathGenerated_, curveSecondStageGenerated_, yawStageGenerated_, actionsSet_, mirrored_, cubeIntaking_, coneIntaking_, placingTimerStarted_, comingDownChargingStation_, taxied_, dumbAutoDocking_;

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
