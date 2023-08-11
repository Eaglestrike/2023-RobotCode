// If you're reading this file, I know it's a mess and terribly structured.
// I swear it was better before, but then we needed more curves and more paths
// and I had to add things in the middle of competitions and had no structure
// to work with and it's rushed etc etc.

#include "AutoPaths.h"

#include "Drivebase/SwervePose.h"
using namespace Poses;

AutoPaths::AutoPaths(SwerveDrive *swerveDrive, TwoJointArm *arm) : swerveDrive_(swerveDrive),
                                                                   arm_(arm)
{
    pointNum_ = 0;
    actionNum_ = 0;
    dumbTimerStarted_ = false;
    actionsSet_ = false;
    slowTraj_ = false;
    pathSet_ = false;
    pathGenerated_ = false;
    curveSecondStageGenerated_ = false;
    yawStageGenerated_ = false;
    mirrored_ = false;
    cubeIntaking_ = false;
    coneIntaking_ = false;
    placingTimerStarted_ = false;
    comingDownChargingStation_ = false;
    taxied_ = false;
    dumbAutoDocking_ = false;
    sendingIt_ = false;
    hitChargeStation_ = false;
    firstCubeArmSafety_ = false;
}

void AutoPaths::setPath(Path path)
{
    path_ = path;
    pointNum_ = 0;
    swervePoints_.clear();
    nextPointReady_ = false;
    dumbTimerStarted_ = false;
    failsafeStarted_ = false;

    switch (path_)
    {
    case BIG_BOY:
    {
        swervePoints_.push_back(SwervePose(0, 1, 0, 0,0,0, 0,0,0, 0)); //YawDist = 0
        swervePoints_.push_back(SwervePose(1, 1, 0, 0,0,0, 0,0,0, 0.1)); //0.1
        swervePoints_.push_back(SwervePose(1, 0, 90, 0,0,0, 0,0,0, 1)); //1
        swervePoints_.push_back(SwervePose(0, 1, 0, 0,0,0, 0,0,0, 10)); //10
        swervePoints_.push_back(SwervePose(0, 0, -90, 0,0,0, 0,0,0, 1)); //1

        break;
    }
    case PRELOADED_CONE_MID:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            yaw = 90;
            if (mirrored_)
            {
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            yaw = -90;
            if (!mirrored_)
            {
                y = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CUBE_MID:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            yaw = 90;
            if (mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            yaw = -90;
            if (!mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_HIGH:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            if (mirrored_)
            {
                yaw = 90;
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
                x += (0.0254 * 2);
            }
            else
            {
                yaw = 90 + 7;
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
                y += (0.0254 * 4);
                x -= (0.0254 * 2);
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            if (!mirrored_)
            {
                yaw = -90 + 7;
                y = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
                y -= (0.0254 * 4);
                x += (0.0254 * 2);
            }
            else
            {
                yaw = -90;
                y = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
                x -= (0.0254 * 2);
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            yaw = 90;
            if (mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            yaw = -90;
            if (!mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            yaw = 90;
            if (mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            yaw = -90;
            if (!mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
            yaw = 90;
            if (mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
            yaw = -90;
            if (!mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case FIRST_CONE_MID:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw1 = -90;
            yaw2 = -90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
                y2 = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw1 = 90;
            yaw2 = 90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
                y2 = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case FIRST_CUBE_HIGH:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw1 = 90;
            yaw2 = 90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y - SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::TOP_CUBE_Y /* - SwerveConstants::CLAW_MID_OFFSET*/ + 0.33;
                yaw2 = 104;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y - SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw1 = -90;
            yaw2 = -90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y + SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y + SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::BOTTOM_CUBE_Y /* + SwerveConstants::CLAW_MID_OFFSET*/ - 0.33;
                yaw2 = -76;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case FIRST_CONE_DOCK:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        y2 = FieldConstants::AUTO_DOCK_Y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_AUTO_DOCK_X;
            yaw1 = -90;
            yaw2 = -90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_AUTO_DOCK_X;
            yaw1 = 90;
            yaw2 = 90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case FIRST_CUBE_DOCK:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        y2 = FieldConstants::AUTO_DOCK_Y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::RED_AUTO_DOCK_X;
            yaw1 = 90;
            yaw2 = 90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw1 = -90;
            yaw2 = -90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case SECOND_CONE:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw2 = -90;
            if (mirrored_)
            {
                yaw1 = -115;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = -65;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw2 = 90;
            if (!mirrored_)
            {
                yaw1 = 115;
                y1 = FieldConstants::TOP_MID_PIECE_Y; // F + 0.3
                y2 = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = 65;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE_MID:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw2 = 90;
            if (mirrored_)
            {
                yaw1 = 45;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y /* - SwerveConstants::CLAW_MID_OFFSET*/ + 0.33;
                yaw2 = 115;
            }
            else
            {
                yaw1 = 135;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw2 = -90;
            if (!mirrored_)
            {
                yaw1 = -45;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = -135;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y /* + SwerveConstants::CLAW_MID_OFFSET*/ - 0.33;
                yaw2 = -65;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE_HIGH:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw2 = 90;
            if (mirrored_)
            {
                yaw1 = 65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = 125;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_SCORING_X;
            yaw2 = -90;
            if (!mirrored_)
            {
                yaw1 = -65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = -115;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CONE_DOCK:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        y2 = FieldConstants::AUTO_DOCK_Y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_AUTO_DOCK_X;
            yaw2 = -90;
            if (mirrored_)
            {
                yaw1 = -135;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw1 = -45;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_AUTO_DOCK_X;
            yaw2 = 90;
            if (!mirrored_)
            {
                yaw1 = 135;
                y1 = FieldConstants::TOP_MID_PIECE_Y; // F + 0.3
            }
            else
            {
                yaw1 = 45;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE_DOCK:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        y2 = FieldConstants::AUTO_DOCK_Y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_AUTO_DOCK_X - 0.33 - 0.33 - 0.8;
            if (mirrored_)
            {
                yaw1 = 45;
                yaw2 = 0; // was 0 then 179.99
                y1 = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw1 = 135;
                yaw2 = 179.99; // Can probably change back to 180 but no time rn
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_AUTO_DOCK_X + 0.33 + 0.33 + 0.8;
            if (!mirrored_)
            {
                yaw1 = -45;
                yaw2 = 0; // was 0, then -179.99
                y1 = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw1 = -135;
                yaw2 = -180;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0.5));

        break;
    }
    case SECOND_CUBE_GRAB:
    {
        double x, y, yaw;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_PIECE_X;
            if (mirrored_)
            {
                yaw = 45;
                y = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw = 135;
                y = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }
        else
        {
            x = FieldConstants::RED_PIECE_X;
            if (!mirrored_)
            {
                yaw = -45;
                y = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw = -135;
                y = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));

        break;
    }
    case AUTO_DOCK:
    {
        double x, y, yaw;
        y = FieldConstants::AUTO_DOCK_Y;
        // yaw = 0;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_AUTO_DOCK_X + 1;
            yaw = 0; // was 90
        }
        else
        {
            x = FieldConstants::RED_AUTO_DOCK_X - 1;
            yaw = 179.99; // was -90
        }
        swervePoints_.push_back(SwervePose(x, y, yaw, 0.5));
        break;
    }
    case TAXI_DOCK_DUMB:
    {
        break;
    }
    case NOTHING:
    {
        break;
    }
    case DRIVE_BACK_DUMB:
    {
        break;
    }
    case WAIT_5_SECONDS:
    {
        break;
    }
    }

    pathSet_ = true;
    pathGenerated_ = false;
    curveSecondStageGenerated_ = false;
    yawStageGenerated_ = false;
}

AutoPaths::Path AutoPaths::getPath()
{
    return path_;
}

void AutoPaths::setActions(Path a1, Path a2, Path a3, Path a4)
{
    actions_.clear();
    actions_.push_back(a1);
    actions_.push_back(a2);
    actions_.push_back(a3);
    actions_.push_back(a4);

    bool overCableBump;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    {
        overCableBump = !mirrored_;
    }
    else
    {
        overCableBump = mirrored_;
    }
    firstCubeArmSafety_ = overCableBump;
    slowTraj_ = overCableBump;

    actionNum_ = 0;
    pointNum_ = 0;
    swervePoints_.clear();
    nextPointReady_ = false;
    dumbTimerStarted_ = false;
    failsafeStarted_ = false;

    actionsSet_ = true;
    pathGenerated_ = false;
    curveSecondStageGenerated_ = false;
    yawStageGenerated_ = false;

    cubeIntaking_ = false;
    coneIntaking_ = false;
    placingTimerStarted_ = false;

    comingDownChargingStation_ = false;
    taxied_ = false;
    dumbAutoDocking_ = false;
    sendingIt_ = false;
    hitChargeStation_ = false;

    switch (a1)
    {
    case PRELOADED_CONE_MID:
    {
        armPosition_ = TwoJointArmProfiles::MID; // MID THING
        forward_ = true;
        break;
    }
    case PRELOADED_CUBE_MID:
    {
        armPosition_ = TwoJointArmProfiles::CUBE_MID;
        forward_ = true;
        break;
    }
    case PRELOADED_CONE_HIGH:
    {
        armPosition_ = TwoJointArmProfiles::HIGH;
        forward_ = true;
        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        armPosition_ = TwoJointArmProfiles::HIGH;
        forward_ = true;
        break;
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        armPosition_ = TwoJointArmProfiles::HIGH;
        forward_ = true;
        break;
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        armPosition_ = TwoJointArmProfiles::MID;
        forward_ = true;
        break;
    }
    case AUTO_DOCK:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
        break;
    }
    case TAXI_DOCK_DUMB:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
        break;
    }
    case NOTHING:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
        break;
    }
    case DRIVE_BACK_DUMB:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
        break;
    }
    case WAIT_5_SECONDS:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
        break;
    }
    default:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        forward_ = true;
    }
    }

    if (a3 == SECOND_CUBE_GRAB || a3 == NOTHING || a3 == DRIVE_BACK_DUMB || a3 == AUTO_DOCK || a3 == FIRST_CUBE_HIGH)
    {
        firstCubeArmSafety_ = true;
        slowTraj_ = true;
    }
}

void AutoPaths::startTimer()
{
    startTime_ = timer_.GetFPGATimestamp().value();
}

void AutoPaths::startAutoTimer()
{
    autoStartTime_ = timer_.GetFPGATimestamp().value();
}

void AutoPaths::setActionsSet(bool actionsSet)
{
    actionsSet_ = actionsSet;
}

void AutoPaths::setPathSet(bool pathSet)
{
    pathSet_ = pathSet;
}

void AutoPaths::periodic()
{
    if (!actionsSet_)
    {
        return;
    }

    if (!pathSet_)
    {
        if (actionNum_ > static_cast<int>(actions_.size()) - 1)
        {
            swerveDrive_->drive(0, 0, 0);
            return;
        }

        setPath(actions_[actionNum_]);
    }

    // frc::SmartDashboard::PutBoolean("actions set", actionsSet_);
    // frc::SmartDashboard::PutBoolean("path set", pathSet_);
    frc::SmartDashboard::PutNumber("action num", actionNum_);
    frc::SmartDashboard::PutNumber("point num", pointNum_);

    double time = timer_.GetFPGATimestamp().value() - startTime_;
    // frc::SmartDashboard::PutNumber("time", time);

    bool pathOver = false;
    bool pointOver = false;
    /*if (path_ == PRELOADED_CONE_MID || path_ == PRELOADED_CUBE_MID || path_ == PRELOADED_CONE_HIGH || path_ == PRELOADED_CUBE_HIGH)
    {
        //swerveDrive_->drive(0, 0, 0);
        if (nextPointReady_)
        {
            pathSet_ = false;
            nextPointReady_ = false;
            ++actionNum_;
            startTimer();
            return;
        }
    }
    else */
    if (path_ != DRIVE_BACK_DUMB && path_ != NOTHING && path_ != WAIT_5_SECONDS && path_ != TAXI_DOCK_DUMB)
    {
        SwervePose pose;
        for (size_t i = pointNum_; i < swervePoints_.size(); ++i)
        {
            // pose = swervePoints_[i].getPose(time, pointOver);
            if (!pathGenerated_ || ((path_ == SECOND_CONE || path_ == SECOND_CUBE_MID || path_ == FIRST_CUBE_HIGH || path_ == AUTO_DOCK || (path_ == FIRST_CONE_DOCK && pointNum_ == 1) || (path_ == FIRST_CUBE_DOCK && pointNum_ == 1) || (path_ == SECOND_CONE_DOCK && pointNum_ == 0) || (path_ == SECOND_CUBE_DOCK /* && pointNum_ == 0*/) || path_ == SECOND_CUBE_GRAB) && (!curveSecondStageGenerated_ || !yawStageGenerated_)))
            {
                SwervePose currPose(swerveDrive_->getX(), swerveDrive_->getY(), swerveDrive_->getYaw(), 0);
                if (path_ == SECOND_CONE || path_ == SECOND_CUBE_MID) // COULDO make the if logic not completely terrible
                {
                    if (!pathGenerated_)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().first);
                        generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().first);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        // double setYaw = swervePoints_[i].yaw;
                        // yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                        // yawStageGenerated_ = true;

                        if (pointNum_ == 0)
                        {
                            double setY;
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                if (mirrored_)
                                {
                                    // setY = 4.8;
                                    setY = FieldConstants::TOP_CONE_Y - 0.18 /* + 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.18 /* - 0.1*/;
                                }
                            }
                            else
                            {
                                if (mirrored_)
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.18 /* - 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.18 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);

                            double setYaw;
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                setYaw = 90;
                            }
                            else
                            {
                                setYaw = -90;
                            }
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                            yawStageGenerated_ = true;
                        }
                        else
                        {
                            double setY = swervePoints_[i].y;
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                            curveSecondStageGenerated_ = true;

                            double setYaw = swervePoints_[i].yaw;
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                            yawStageGenerated_ = true;
                        }
                    }

                    // Pose1D yProfile = yTraj_.getProfile();
                    Pose1D yProfile = getYProfile();

                    bool curveReady = false;
                    if (pointNum_ == 0)
                    {
                        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                        {
                            curveReady = (swerveDrive_->getX() > 4.8514); // 2.921
                        }
                        else
                        {
                            curveReady = (swerveDrive_->getX() < 11.688318); // 13.621893
                        }
                    }
                    else
                    {
                        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                        {
                            curveReady = (swerveDrive_->getX() < 4.8514);
                        }
                        else
                        {
                            curveReady = (swerveDrive_->getX() > 11.688318);
                        }
                    }

                    if ((!curveSecondStageGenerated_) && (/*pointNum_ == 1 || */ (pointNum_ == 0 && curveReady && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.5 && isStationary(yProfile))))
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                        yawStageGenerated_ = true;
                    }
                    else if ((!curveSecondStageGenerated_) && ((pointNum_ == 1 && curveReady && isStationary(yProfile))))
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;
                    }

                    // if (curveSecondStageGenerated_ && !yawStageGenerated_ && pointNum_ == 1)
                    // {

                    //     if (get<0>(yProfile) == 0 && get<1>(yProfile) == 0)
                    //     {
                    //         double setYaw = swervePoints_[i].yaw;
                    //         yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                    //         yawStageGenerated_ = true;
                    //     }
                    // }
                }
                else if (path_ == FIRST_CUBE_HIGH)
                {
                    if (!pathGenerated_)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().first);
                        generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().first);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0);
                        yawStageGenerated_ = true;

                        if (/*pointNum_ == 0*/ true)
                        {
                            double setY;
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                if (mirrored_)
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 - 0.0254 * 2 /* + 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                            }
                            else
                            {
                                if (mirrored_)
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 - 0.0254 * 2 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        }
                        // else
                        // {
                        //     double setY = swervePoints_[i].y;
                        //     yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        //     curveSecondStageGenerated_ = true;
                        // }
                    }

                    // Pose1D yProfile = yTraj_.getProfile();
                    Pose1D yProfile = getYProfile();

                    if ((!curveSecondStageGenerated_ && isStationary(yProfile)))
                    {
                        bool curveReady = false;
                        if (pointNum_ == 0)
                        {
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                curveReady = (swerveDrive_->getX() > 2.921);
                            }
                            else
                            {
                                curveReady = (swerveDrive_->getX() < 13.621893);
                            }
                        }
                        else
                        {
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                curveReady = (swerveDrive_->getX() < 4.8514 + 0.5);
                            }
                            else
                            {
                                curveReady = (swerveDrive_->getX() > 11.688318 - 0.5);
                            }
                        }

                        if (curveReady)
                        {
                            double setY = swervePoints_[i].y;
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                            curveSecondStageGenerated_ = true;
                        }
                    }
                }
                else if (path_ == AUTO_DOCK)
                {
                    if (!pathGenerated_)
                    {
                        double setX;
                        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                        {
                            setX = FieldConstants::BLUE_SCORING_X + 0.3;
                        }
                        else
                        {
                            setX = FieldConstants::RED_SCORING_X - 0.3;
                        }
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);

                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(currPose.y, setY, swerveDrive_->getXYVel().second);
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();
                        yawStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.2)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                        yawStageGenerated_ = true;
                    }
                }
                else if ((path_ == FIRST_CONE_DOCK || path_ == FIRST_CUBE_DOCK) && pointNum_ == 1)
                {
                    if (!pathGenerated_)
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(currPose.y, setY, swerveDrive_->getXYVel().second);
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageGenerated_ = true;
                    }
                }
                else if ((path_ == SECOND_CONE_DOCK || path_ == SECOND_CUBE_DOCK || path_ == SECOND_CUBE_GRAB) /* && pointNum_ == 0*/)
                {
                    if (pointNum_ == 0)
                    {
                        if (!pathGenerated_)
                        {
                            double setX = swervePoints_[i].x;
                            // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().first);
                            generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().first);
                            curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                            // double setYaw = swervePoints_[i].yaw;
                            // yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                            double setYaw;
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                setYaw = 90;
                            }
                            else
                            {
                                setYaw = -90;
                            }
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                            yawStageGenerated_ = true;

                            // double setY = swervePoints_[i].y;
                            double setY;
                            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                            {
                                if (mirrored_)
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 /* + 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                            }
                            else
                            {
                                if (mirrored_)
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);

                            curveSecondStageGenerated_ = false;
                            pathGenerated_ = true;
                        }

                        // frc::SmartDashboard::PutBoolean("GEN", curveSecondStageGenerated_);
                        // frc::SmartDashboard::PutBoolean("PATH", pathGenerated_);
                        // frc::SmartDashboard::PutNumber("TIME", timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_);
                        // Pose1D yProfile = yTraj_.getProfile();
                        Pose1D yProfile = getYProfile();

                        bool curveReady = false;
                        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                        {
                            curveReady = (swerveDrive_->getX() > 4.8514); // 2.921
                        }
                        else
                        {
                            curveReady = (swerveDrive_->getX() < 11.688318); // 13.621893
                        }
                        if ((!curveSecondStageGenerated_) /* && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.5*/ && curveReady && isStationary(yProfile))
                        {
                            double setY = swervePoints_[i].y;
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                            curveSecondStageGenerated_ = true;

                            double setYaw = swervePoints_[i].yaw;
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                        }
                    }
                    else
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // TODO yaw vel
                        yawStageGenerated_ = true;

                        pathGenerated_ = true;
                    }
                }
                else
                {
                    currPath_.reset();
                    currPath_.addPoint(currPose);
                    currPath_.addPoint(swervePoints_[i]);
                    // frc::SmartDashboard::PutNumber("WANTED YAW", swervePoints_[i].yaw);
                    // frc::SmartDashboard::PutNumber("CURR YAW", currPose.yaw);

                    currPath_.generateTrajectory(false);

                    pathGenerated_ = true;
                }
            }

            if (path_ == SECOND_CONE || path_ == SECOND_CUBE_MID)
            {
                // Pose1D xProfile = xTraj_.getProfile();
                Pose1D xProfile = getXProfile();
                // Pose1D yProfile = yTraj_.getProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();
                // if (curveSecondStageGenerated_)
                // {
                //     // yProfile = yTraj_.getProfile();
                //     //  frc::SmartDashboard::PutNumber("WY", get<2>(yProfile));
                //     //  frc::SmartDashboard::PutNumber("WYV", get<1>(yProfile));
                //     //  frc::SmartDashboard::PutNumber("WYA", get<0>(yProfile));
                //     //  frc::SmartDashboard::PutNumber("YVel", swerveDrive_->getXYVel().second);
                // }
                // if (yawStageGenerated_)
                // {
                //     yawProfile = yawTraj_.getProfile();
                // }
                pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                
                if (isStationary(pose))
                {
                    pointOver = true;
                }
            }
            else if (path_ == FIRST_CUBE_HIGH)
            {
                // Pose1D xProfile = xTraj_.getProfile();
                Pose1D xProfile = getXProfile();
                // Pose1D yProfile = yTraj_.getProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();

                pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                if (isStationary(pose))
                {
                    pointOver = true;
                }
            }
            else if (path_ == AUTO_DOCK)
            {
                // double heldX = (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) ? FieldConstants::BLUE_SCORING_X + 0.05 : FieldConstants::RED_SCORING_X - 0.05;
                // Pose1D xProfile = xTraj_.getProfile(); // held x was used here to prevent drift
                Pose1D xProfile = getXProfile();
                // Pose1D yProfile = yTraj_.getProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();
                // if (curveSecondStageGenerated_)
                // {
                //     xProfile = xTraj_.getProfile();
                // }
                // pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));

                // if (get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
                // {
                //     pointOver = true;
                // }

                if (curveSecondStageGenerated_ && !hitChargeStation_)
                {
                    double xVel;
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        xVel = SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    else
                    {
                        xVel = -SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    pose = SwerveFromPose1D({swerveDrive_->getX(), xVel, 0}, yProfile, yawProfile);

                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        if (tilt < -5)
                        {
                            // pointOver = true;
                            // return;
                            hitChargeStation_ = true;
                        }
                    }
                    else
                    {
                        if (tilt > 5)
                        {
                            // pointOver = true;
                            // return;
                            hitChargeStation_ = true;
                        }
                    }
                }
                else
                {
                    pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                }
            }
            else if ((path_ == SECOND_CONE_DOCK || path_ == SECOND_CUBE_DOCK || path_ == SECOND_CUBE_GRAB) /* && pointNum_ == 0*/)
            {
                // Pose1D xProfile = xTraj_.getProfile();
                Pose1D xProfile = getXProfile();
                // Pose1D yProfile = yTraj_.getProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();
                // if (curveSecondStageGenerated_)
                // {
                //     yProfile = yTraj_.getProfile();
                // }
                if (pointNum_ == 0)
                {
                    pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                    if (isStationary(pose))
                    {
                        pointOver = true;
                    }
                }
                else if (!hitChargeStation_)
                {
                    double xVel;
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        xVel = -SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    else
                    {
                        xVel = SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    // frc::SmartDashboard::PutNumber("Y DOCK VEL", get<1>(yProfile));
                    // frc::SmartDashboard::PutNumber("YAW DOCK VEL", get<1>(yawProfile));
                    pose = SwerveFromPose1D({swerveDrive_->getX(), xVel, 0}, yProfile, yawProfile);
                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    frc::SmartDashboard::PutNumber("DTilt", tilt);
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        if (tilt > 5)
                        {
                            // pointOver = true;
                            // return;
                            hitChargeStation_ = true;
                        }
                    }
                    else
                    {
                        if (tilt < -5)
                        {
                            // pointOver = true;
                            // return;
                            hitChargeStation_ = true;
                        }
                    }

                    frc::SmartDashboard::PutBoolean("Hit Charge Station", false);
                    frc::SmartDashboard::PutBoolean("Sending it Fast", false);
                    frc::SmartDashboard::PutBoolean("Sending it Medium", false);
                }
                else
                {
                    frc::SmartDashboard::PutBoolean("Hit Charge Station", true);
                }
            }
            else if ((path_ == FIRST_CONE_DOCK && pointNum_ == 1) || (path_ == FIRST_CUBE_DOCK && pointNum_ == 1))
            {
                double heldX = (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) ? FieldConstants::BLUE_PIECE_X : FieldConstants::RED_PIECE_X;
                Pose1D xProfile = {0, 0, heldX}; // swerveDrive_->getX();
                // Pose1D yProfile = yTraj_.getProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();
                if (curveSecondStageGenerated_)
                {
                    // xProfile = xTraj_.getProfile();
                    xProfile = getXProfile();
                }
                pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                if (isStationary(pose))
                {
                    pointOver = true;
                }
            }
            else
            {
                pose = currPath_.getPose(time, pointOver);
                // frc::SmartDashboard::PutNumber("T", time);
            }

            if (!pointOver)
            {
                break;
            }
            else
            {
                if (nextPointReady_ && i == swervePoints_.size() - 1)
                {
                    pathSet_ = false;
                    nextPointReady_ = false;
                    ++actionNum_;
                    // pathGenerated_ = false;
                    // curveSecondStageGenerated_ = false;
                    // yawStageGenerated_ = false;
                    startTimer();
                    return;
                }
                else if (i == swervePoints_.size() - 1)
                {
                    pathOver = true;
                }
                else if (nextPointReady_ && i != swervePoints_.size() - 1)
                {
                    nextPointReady_ = false;
                    ++pointNum_;
                    startTimer();
                    pathGenerated_ = false;
                    curveSecondStageGenerated_ = false;
                    yawStageGenerated_ = false;
                    time = timer_.GetFPGATimestamp().value() - startTime_;
                }
                else
                {
                    break;
                }
            }
        }

        if (!isZero(pose))
        {
            if (timer_.GetFPGATimestamp().value() - autoStartTime_ > 14.9 && (path_ == SECOND_CUBE_DOCK || path_ == FIRST_CUBE_DOCK || path_ == SECOND_CONE_DOCK || path_ == FIRST_CONE_DOCK || path_ == AUTO_DOCK))
            {
                swerveDrive_->lockWheels();
            }
            else if (!pointOver || pointNum_ != 1 || path_ != SECOND_CUBE_DOCK)
            {
                // frc::SmartDashboard::PutNumber("x thing", pose->getXVel());
                // frc::SmartDashboard::PutNumber("y thing", pose->getYVel());
                // frc::SmartDashboard::PutNumber("yaw thing", pose->getYawVel());
                swerveDrive_->drivePose(pose);
            }
            // delete pose;
        }
    }
    else if (path_ == WAIT_5_SECONDS)
    {
        if (nextPointReady_)
        {
            pathSet_ = false;
            nextPointReady_ = false;
            ++actionNum_;
            startTimer();
            return;
        }
    }
    else
    {
        if (!dumbTimerStarted_)
        {
            timer_.Stop();
            timer_.Reset();
            timer_.Start();
            dumbTimerStarted_ = true;
        }
    }

    switch (path_)
    {
    case BIG_BOY:
    {
        if (pathOver)
        {
            // Do a thing
        }

        if (pointOver)
        {
            switch (pointNum_)
            {
            case 0:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2)
                {
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeStarted_ = false;
                    nextPointReady_ = true;
                }
                break;
            }
            case 1:
            {
                if (!failsafeStarted_)
                {
                    failsafeStarted_ = true;
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeTimer_.Start();
                }

                if (failsafeTimer_.Get().value() > 2)
                {
                    failsafeTimer_.Stop();
                    failsafeTimer_.Reset();
                    failsafeStarted_ = false;
                    nextPointReady_ = true;
                }
                break;
            }
            case 2:
            {
                nextPointReady_ = true;
                break;
            }
            case 3:
            {
                nextPointReady_ = true;
                break;
            }
            case 4:
            {
                nextPointReady_ = true;
                break;
            }
            }
        }

        break;
    }
    case PRELOADED_CONE_MID:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        coneIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::MID;

        if (arm_->getPosition() == TwoJointArmProfiles::MID && arm_->getState() == TwoJointArm::HOLDING_POS) // MID THING
        {
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            // if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.2)
            // {
            //     clawOpen_ = true;
            // }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.3)
            {
                // pointOver = true;
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else
        {
            clawOpen_ = false;
        }

        break;
    }
    case PRELOADED_CUBE_MID:
    {
        forward_ = true;
        clawOpen_ = true;
        armPosition_ = TwoJointArmProfiles::CUBE_MID;

        if (arm_->getPosition() == TwoJointArmProfiles::CUBE_MID && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.4)
            {
                pointOver = true;
                nextPointReady_ = true;
            }
        }

        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
        }

        break;
    }
    case PRELOADED_CONE_HIGH:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        coneIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::HIGH && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            // if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.2)
            // {
            //     clawOpen_ = true;
            // }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.3)
            {
                // pointOver = true;
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else
        {
            clawOpen_ = false;
        }

        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        forward_ = true;
        clawOpen_ = true;
        armPosition_ = TwoJointArmProfiles::CUBE_HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::CUBE_HIGH && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.4)
            {
                pointOver = true;
                nextPointReady_ = true;
            }
        }

        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
        }

        break;
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        coneIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::HIGH && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            // if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.2)
            // {
            //     clawOpen_ = true;
            // }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.3)
            {
                // pointOver = true;
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else
        {
            clawOpen_ = false;
        }

        break;
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        coneIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::MID;

        if (arm_->getPosition() == TwoJointArmProfiles::MID && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_)
            {
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            // if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.2)
            // {
            //     clawOpen_ = true;
            // }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.3)
            {
                // pointOver = true;
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else
        {
            clawOpen_ = false;
        }

        break;
    }
    case FIRST_CONE_MID:
    {
        if (pointOver && pointNum_ == 0)
        {
            nextPointReady_ = true;
        }

        // TODO place cone
        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 1)
        {
            nextPointReady_ = true;
        }

        break;
    }
    case FIRST_CUBE_HIGH:
    {
        if (pointNum_ == 0)
        {
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (firstCubeArmSafety_)
            {
                bool armReady;
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                {
                    armReady = (swerveDrive_->getX() < 2.921 - 0.2);
                }
                else
                {
                    armReady = (swerveDrive_->getX() > 13.621893 + 0.2);
                }
                if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && !armReady)
                {
                    cubeIntaking_ = false;
                    forward_ = false; // SAFETY WAS TRUE
                    // if (arm_->isForward())
                    // {
                    //     if (arm_->getPosition() != TwoJointArmProfiles::CUBE_MID)
                    //     {
                    armPosition_ = TwoJointArmProfiles::STOWED;
                    // }
                    // }
                }
                else if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && armReady)
                {
                    cubeIntaking_ = false;
                    forward_ = true;
                    armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
                }
                else
                {
                    forward_ = false;
                    clawOpen_ = true;
                    cubeIntaking_ = true;
                }
            }
            else
            {
                if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2)
                {
                    cubeIntaking_ = false;
                    forward_ = true;
                    armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
                }
                else
                {
                    forward_ = false;
                    clawOpen_ = true;
                    cubeIntaking_ = true;
                }
            }

            if (arm_->getPosition() == TwoJointArmProfiles::CUBE_HIGH && arm_->getState() == TwoJointArm::HOLDING_POS && pointOver)
            {
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
                if (!placingTimerStarted_)
                {
                    placingStartTime_ = timer_.GetFPGATimestamp().value();
                    placingTimerStarted_ = true;
                }

                if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.4)
                {
                    nextPointReady_ = true;
                    placingTimerStarted_ = false;
                }
            }
        }

        break;
    }
    case FIRST_CONE_DOCK:
    {
        break;
    }
    case FIRST_CUBE_DOCK:
    {
        if (pointNum_ == 0)
        {
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2)
            {
                cubeIntaking_ = false;
                forward_ = false;
            }

            if (pointOver && timer_.GetFPGATimestamp().value() - autoStartTime_ < 14.9)
            {
                double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    swerveDrive_->lockWheels();
                }
                else
                {
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive(output, 0, 0);
                }
            }
        }

        break;
    }
    case SECOND_CONE:
    {
        if (pointOver && pointNum_ != 3)
        {
            nextPointReady_ = true;
        }

        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 3)
        {
            nextPointReady_ = true;
        }
        break;
    }
    case SECOND_CUBE_MID:
    {
        if (pointNum_ == 0)
        {
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            bool armReady;
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
            {
                armReady = (swerveDrive_->getX() < 2.921 - 0.2);
            }
            else
            {
                armReady = (swerveDrive_->getX() > 13.621893 + 0.2);
            }
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && !armReady)
            {
                cubeIntaking_ = false;
                forward_ = false; // SAFETY WAS TRUE
                // if (arm_->isForward())
                // {
                //     if (arm_->getPosition() != TwoJointArmProfiles::CUBE_MID)
                //     {
                armPosition_ = TwoJointArmProfiles::STOWED; // SAFETY WAS CUBE MID
                // }
                // }
            }
            else if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && armReady)
            {
                cubeIntaking_ = false;
                forward_ = true;
                armPosition_ = TwoJointArmProfiles::CUBE_MID;
            }
            else
            {
                forward_ = false;
                clawOpen_ = true;
                cubeIntaking_ = true;
            }

            if (arm_->getPosition() == TwoJointArmProfiles::CUBE_MID && arm_->getState() == TwoJointArm::HOLDING_POS && pointOver)
            {
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED - 3;
                if (!placingTimerStarted_)
                {
                    placingStartTime_ = timer_.GetFPGATimestamp().value();
                    placingTimerStarted_ = true;
                }

                if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.4)
                {
                    nextPointReady_ = true;
                    placingTimerStarted_ = false;
                }
            }
        }

        break;

        // // TODO see if curves are the same
        // if (pointOver && pointNum_ != 3)
        // {
        //     nextPointReady_ = true;
        // }

        // // TODO place cone
        // // if done placing
        // if (/*done placing &&*/ pointOver && pointNum_ == 3)
        // {
        //     nextPointReady_ = true;
        // }
        // break;
    }
    case SECOND_CUBE_HIGH:
    {
        break;
    }
    case SECOND_CONE_DOCK:
    {
        break;
    }
    case SECOND_CUBE_DOCK:
    {
        if (pointNum_ == 0)
        {
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2)
            {
                cubeIntaking_ = false;
                forward_ = false;
                armPosition_ = TwoJointArmProfiles::STOWED;
            }

            if (hitChargeStation_ && timer_.GetFPGATimestamp().value() - autoStartTime_ < 14.9)
            {
                if (!sendingIt_)
                {
                    sendingItTime_ = timer_.GetFPGATimestamp().value();
                    sendingIt_ = true;
                }
                double time = timer_.GetFPGATimestamp().value() - sendingItTime_;
                if (time < SwerveConstants::SENDING_IT_TIME)
                {
                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        if (abs(tilt) < SwerveConstants::MIN_TILT_ON_STATION)
                        {
                            frc::SmartDashboard::PutBoolean("Sending it Fast", true);
                            frc::SmartDashboard::PutBoolean("Sending it Medium", false);
                            swerveDrive_->drive(-SwerveConstants::SENDING_IT_FAST_SPEED, 0, 0);
                        }
                        else
                        {
                            frc::SmartDashboard::PutBoolean("Sending it Fast", false);
                            frc::SmartDashboard::PutBoolean("Sending it Medium", true);
                            swerveDrive_->drive(-SwerveConstants::SENDING_IT_MED_SPEED, 0, 0);
                        }
                    }
                    else
                    {
                        if (abs(tilt) < SwerveConstants::MIN_TILT_ON_STATION)
                        {
                            frc::SmartDashboard::PutBoolean("Sending it Fast", true);
                            frc::SmartDashboard::PutBoolean("Sending it Medium", false);
                            swerveDrive_->drive(SwerveConstants::SENDING_IT_FAST_SPEED, 0, 0);
                        }
                        else
                        {
                            frc::SmartDashboard::PutBoolean("Sending it Fast", false);
                            frc::SmartDashboard::PutBoolean("Sending it Medium", true);
                            swerveDrive_->drive(SwerveConstants::SENDING_IT_MED_SPEED, 0, 0);
                        }
                    }
                }
                else
                {
                    frc::SmartDashboard::PutBoolean("Sending it Fast", false);
                    frc::SmartDashboard::PutBoolean("Sending it Medium", false);
                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                    {
                        frc::SmartDashboard::PutBoolean("Balanced", true);
                        swerveDrive_->lockWheels();
                    }
                    else
                    {
                        frc::SmartDashboard::PutBoolean("Balanced", false);
                        double output = -SwerveConstants::AUTOKTILT * tilt;
                        swerveDrive_->drive(output, 0, 0);
                    }
                }
            }
            else if(timer_.GetFPGATimestamp().value() - autoStartTime_ > 14.9)
            {
                swerveDrive_->lockWheels();
            }
        }

        break;
    }
    case SECOND_CUBE_GRAB:
    {
        if (timer_.GetFPGATimestamp().value() - autoStartTime_ < 15.0 - 1.126)
        {
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
        }
        else
        {
            armPosition_ = TwoJointArmProfiles::STOWED;
            forward_ = false;
            cubeIntaking_ = false;
            clawOpen_ = true;
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
        }
        break;
    }
    case AUTO_DOCK:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        clawOpen_ = false;
        wheelSpeed_ = 0;
        if (hitChargeStation_ && timer_.GetFPGATimestamp().value() - autoStartTime_ < 14.9)
        {
            if (!sendingIt_)
            {
                sendingItTime_ = timer_.GetFPGATimestamp().value();
                sendingIt_ = true;
            }
            double time = timer_.GetFPGATimestamp().value() - sendingItTime_;
            if (time < SwerveConstants::SENDING_IT_TIME)
            {
                double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                {
                    if (abs(tilt) < 12)
                    {
                        swerveDrive_->drive(SwerveConstants::SENDING_IT_FAST_SPEED, 0, 0);
                    }
                    else
                    {
                        swerveDrive_->drive(SwerveConstants::SENDING_IT_MED_SPEED, 0, 0);
                    }
                }
                else
                {
                    if (abs(tilt) < 12)
                    {
                        swerveDrive_->drive(-SwerveConstants::SENDING_IT_FAST_SPEED, 0, 0);
                    }
                    else
                    {
                        swerveDrive_->drive(-SwerveConstants::SENDING_IT_MED_SPEED, 0, 0);
                    }
                }
            }
            else
            {
                double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    swerveDrive_->lockWheels();
                }
                else
                {
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive(output, 0, 0);
                }
            }
        }
        break;
    }
    case TAXI_DOCK_DUMB:
    {
        forward_ = true;
        clawOpen_ = false;
        wheelSpeed_ = 0;
        armPosition_ = TwoJointArmProfiles::STOWED;

        double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
        double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
        double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
        double tilt = pitch * sin(ang) - roll * cos(ang);

        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (tilt > 5)
            {
                comingDownChargingStation_ = true;
            }
        }
        else
        {
            if (tilt < -5)
            {
                comingDownChargingStation_ = true;
            }
        }

        if (!comingDownChargingStation_)
        {
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
            {
                swerveDrive_->drive(0.5, 0, 0);
            }
            else
            {
                swerveDrive_->drive(-0.5, 0, 0);
            }
        }
        else if (comingDownChargingStation_ && !taxied_)
        {
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
            {
                swerveDrive_->drive(0.35, 0, 0);
            }
            else
            {
                swerveDrive_->drive(-0.35, 0, 0);
            }

            taxied_ = (abs(tilt) < 3);
        }
        else
        {
            if (!dumbAutoDocking_)
            {
                if (abs(tilt) > 7)
                {
                    dumbAutoDocking_ = true;
                }
                else
                {
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        swerveDrive_->drive(-0.35, 0, 0);
                    }
                    else
                    {
                        swerveDrive_->drive(0.35, 0, 0);
                    }
                }
            }

            if (dumbAutoDocking_)
            {
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    swerveDrive_->lockWheels();
                }
                else
                {
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive(output, 0, 0);
                }
            }
        }
        break;
    }
    case NOTHING:
    {
        swerveDrive_->lockWheels();
        break;
    }
    case DRIVE_BACK_DUMB:
    {
        // if (!failsafeStarted_)
        // {
        //     failsafeStarted_ = true;
        //     failsafeTimer_.Stop();
        //     failsafeTimer_.Reset();
        //     failsafeTimer_.Start();
        // }

        // if (failsafeTimer_.Get().value() < 2)
        // {
        //     swerveDrive_->drive(0, 0.2, 0);
        // }
        // else
        // {
        //     swerveDrive_->drive(0, 0, 0);
        // }

        if (timer_.Get().value() < 2)
        {
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
            {
                swerveDrive_->drive(0.2, 0, 0);
            }
            else
            {
                swerveDrive_->drive(-0.2, 0, 0);
            }
        }
        else
        {
            swerveDrive_->drive(0, 0, 0);
        }

        break;
    }
    case WAIT_5_SECONDS:
    {
        swerveDrive_->lockWheels();
        if (!failsafeStarted_)
        {
            failsafeStarted_ = true;
            failsafeTimer_.Reset();
            failsafeTimer_.Start();
        }

        if (failsafeTimer_.Get().value() > 5)
        {
            failsafeStarted_ = false;
            nextPointReady_ = true;
        }
    }
    }
}

void AutoPaths::setGyros(double yaw, double pitch, double roll)
{
    yaw_ = yaw;
    pitch_ = pitch;
    roll_ = roll;
}

double AutoPaths::initYaw()
{
    // switch (path_)
    // {
    // case BIG_BOY:
    // {
    //     return 0;
    //     break;
    // }
    // }

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    {
        return -90;
    }
    else
    {
        return 90;
    }
}

std::pair<double, double> AutoPaths::initPos()
{
    switch (actions_[0])
    {
    case PRELOADED_CONE_MID:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case PRELOADED_CUBE_MID:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case PRELOADED_CONE_HIGH:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case PRELOADED_CUBE_HIGH:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            x = FieldConstants::RED_SCORING_X;
        }

        double y;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            if (mirrored_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        return {x, y};
    }
    case AUTO_DOCK:
    {
        double y = FieldConstants::AUTO_DOCK_Y;
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            // x = FieldConstants::BLUE_AUTO_DOCK_X;
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            // x = FieldConstants::RED_AUTO_DOCK_X;
            x = FieldConstants::RED_SCORING_X;
        }
        return {x, y};
    }
    case TAXI_DOCK_DUMB:
    {
        double y = FieldConstants::AUTO_DOCK_Y;
        double x;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            // x = FieldConstants::BLUE_AUTO_DOCK_X;
            x = FieldConstants::BLUE_SCORING_X;
        }
        else
        {
            // x = FieldConstants::RED_AUTO_DOCK_X;
            x = FieldConstants::RED_SCORING_X;
        }
        return {x, y};
    }
    case NOTHING:
    {
        return {0, 0};
    }
    case DRIVE_BACK_DUMB:
    {
        return {0, 0};
    }
    default:
    {
        return {0, 0};
    }
    }
}

int AutoPaths::pointNum()
{
    return pointNum_;
}

void AutoPaths::setMirrored(bool mirrored)
{
    mirrored_ = mirrored;
}

bool AutoPaths::getClawOpen()
{
    return clawOpen_;
}
bool AutoPaths::getForward()
{
    return forward_;
}
double AutoPaths::getWheelSpeed()
{
    return wheelSpeed_;
}
TwoJointArmProfiles::Positions AutoPaths::getArmPosition()
{
    return armPosition_;
}

bool AutoPaths::cubeIntaking()
{
    return cubeIntaking_;
}

bool AutoPaths::coneIntaking()
{
    return coneIntaking_;
}

void AutoPaths::generateXTraj(double pos, double setPos, double vel)
{
    if (slowTraj_)
    {
        xSlowTraj_.generateTrajectory(pos, setPos, vel);
    }
    else
    {
        xTraj_.generateTrajectory(pos, setPos, vel);
    }
}
void AutoPaths::generateYTraj(double pos, double setPos, double vel)
{
    if (slowTraj_)
    {
        ySlowTraj_.generateTrajectory(pos, setPos, vel);
    }
    else
    {
        yTraj_.generateTrajectory(pos, setPos, vel);
    }
}

Pose1D AutoPaths::getXProfile()
{
    if (slowTraj_)
    {
        return xSlowTraj_.getProfile();
    }
    else
    {
        return xTraj_.getProfile();
    }
}
Pose1D AutoPaths::getYProfile()
{
    if (slowTraj_)
    {
        return ySlowTraj_.getProfile();
    }
    else
    {
        return yTraj_.getProfile();
    }
}