#pragma once

#include <vector>

#include <frc/Timer.h>

#include "GeneralConstants.h"

#include "Drivebase/SwerveDrive.h"
#include "Drivebase/SwervePath.h"

#include "Arm/TwoJointArm.h"
#include "Arm/TwoJointArmProfiles.h"

#include "Helpers/GeneralPoses.h"

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
            PRELOADED_CONE_HIGH_MIDDLE,
            PRELOADED_CONE_MID_MIDDLE,
            FIRST_CUBE_HIGH,
            FIRST_CUBE_DOCK,
            SECOND_CUBE_MID,
            SECOND_CUBE_HIGH,
            SECOND_CUBE_DOCK,
            SECOND_CUBE_GRAB,
            AUTO_DOCK,
            NO_TAXI_DOCK_DUMB,
            TAXI_DOCK_DUMB,
            ROTATE_ONLY_ROTATE,
            NOTHING,
            DRIVE_BACK_DUMB,
            WAIT_5_SECONDS
        };
        AutoPaths(SwerveDrive* swerveDrive, TwoJointArm* arm);
        void setActions(Path a1, Path a2, Path a3, Path a4, bool slow);
        std::vector<Path> getActions();

        void startTimer();
        void startAutoTimer();
        void setActionsSet(bool actionsSet);
        void setPathSet(bool pathSet);

        void periodic();
        void setGyros(double yaw, double pitch, double roll);
        double initYaw();
        Point initPos();

        int pointNum();

        void setMirrored(bool mirrored);

        bool getClawOpen();
        bool getForward();
        double getWheelSpeed();
        TwoJointArmProfiles::Positions getArmPosition();
        bool cubeIntaking();

        void generateXTraj(double pos, double setPos, double vel);
        void generateYTraj(double pos, double setPos, double vel);

        Poses::Pose1D getXProfile();
        Poses::Pose1D getYProfile();

    private:
        std::vector<Path> actions_;
        Path path_;
        SwerveDrive* swerveDrive_;
        TwoJointArm* arm_;

        TrajectoryCalc xTraj_{{SwerveConstants::MAX_LV * 1.0, SwerveConstants::MAX_LA * 1.0, 0, 0, 0, 0}};
        TrajectoryCalc yTraj_{{SwerveConstants::MAX_LV * 1.0, SwerveConstants::MAX_LA * 1.0, 0, 0, 0, 0}};
        TrajectoryCalc yawTraj_{{SwerveConstants::MAX_AV * 0.3, SwerveConstants::MAX_AA * 0.3, 0, 0, 0, 0}};

        frc::Timer timer_;
        frc::Timer failsafeTimer_;
        double startTime_, curveSecondStageStartTime_, placingStartTime_, yaw_, pitch_, roll_, autoStartTime_, sendingItTime_;
        bool nextPointReady_, failsafeStarted_, dumbTimerStarted_, pathSet_, pathGenerated_, curveSecondStageGenerated_, yawStageGenerated_, actionsSet_, left_, cubeIntaking_, placingTimerStarted_, comingDownChargingStation_, taxied_, dumbAutoDocking_, sendingIt_, firstCubeArmSafety_, hitChargeStation_;

        double armStart_ = 0.0; //Time started arm extension
        bool armPlacing_ = false;
        
        double taxiStart = 0.0; //Drive out a bit taxi
        bool taxiDriving_ = false;
        bool onCharge_ = false;

        bool isBlue_;

        //std::vector<SwervePath> swervePaths_;
        int actionNum_;
        std::vector<Poses::SwervePose> swervePoints_;
        SwervePath currPath_{SwerveConstants::MAX_LA, SwerveConstants::MAX_LV, SwerveConstants::MAX_AA, SwerveConstants::MAX_AV};
        int pointNum_;

        void setPath(Path path);
        Path getPath();

        bool clawOpen_, forward_;
        double wheelSpeed_;
        TwoJointArmProfiles::Positions armPosition_;
};
