#include "AutoPaths.h"

AutoPaths::AutoPaths(SwerveDrive *swerveDrive, TwoJointArm *arm) : swerveDrive_(swerveDrive),
                                                                   arm_(arm)
{
    pointNum_ = 0;
    actionNum_ = 0;
    dumbTimerStarted_ = false;
    actionsSet_ = false;
    pathSet_ = false;
    pathGenerated_ = false;
    curveSecondStageGenerated_ = false;
    yawStageGenerated_ = false;
    mirrored_ = false;
    cubeIntaking_ = false;
    coneIntaking_ = false;
    placingTimerStarted_ = false;
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
        swervePoints_.push_back(SwervePose(0, 1, 0, 0));
        swervePoints_.push_back(SwervePose(1, 1, 0, 0.1));
        swervePoints_.push_back(SwervePose(1, 0, 90, 1));
        swervePoints_.push_back(SwervePose(0, 1, 0, 10));
        swervePoints_.push_back(SwervePose(0, 0, -90, 1));

        break;
    }
    case PRELOADED_CONE_MID:
    {
        break;
    }
    case PRELOADED_CUBE_MID:
    {
        break;
    }
    case PRELOADED_CONE_HIGH:
    {
        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
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
                y2 = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y1 = FieldConstants::TOP_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
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
                y1 = FieldConstants::TOP_PIECE_Y - 0.305;
                y2 = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y- 0.305;
                y2 = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y2 = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = 65;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                yaw1 = 65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = 125;
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
                yaw1 = -65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = -115;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                yaw1 = -115;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
            x2 = FieldConstants::BLUE_AUTO_DOCK_X;
            yaw2 = 90;
            if (mirrored_)
            {
                yaw1 = 45;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw1 = 135;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X;
            x2 = FieldConstants::RED_AUTO_DOCK_X;
            yaw2 = -90;
            if (!mirrored_)
            {
                yaw1 = -45;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
            }
            else
            {
                yaw1 = -135;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case AUTO_DOCK:
    {
        double x, y, yaw;
        y = FieldConstants::AUTO_DOCK_Y;
        yaw = 0;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_AUTO_DOCK_X;
            //yaw = 90;
        }
        else
        {
            x = FieldConstants::RED_AUTO_DOCK_X;
            //yaw = -90;
        }
        swervePoints_.push_back(SwervePose(x, y, yaw, 0.5));
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

    switch (a1)
    {
    case PRELOADED_CONE_MID:
    {
        armPosition_ = TwoJointArmProfiles::MID;
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
        armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
        forward_ = true;
        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        armPosition_ = TwoJointArmProfiles::HIGH;
        forward_ = true;
        break;
    }
    case AUTO_DOCK:
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
}

void AutoPaths::startTimer()
{
    startTime_ = timer_.GetFPGATimestamp().value();
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
        if (actionNum_ > actions_.size() - 1)
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
    if (path_ == PRELOADED_CONE_MID || path_ == PRELOADED_CUBE_MID || path_ == PRELOADED_CONE_HIGH || path_ == PRELOADED_CUBE_HIGH)
    {
        if (nextPointReady_)
        {
            pathSet_ = false;
            nextPointReady_ = false;
            ++actionNum_;
            swerveDrive_->drive(0, 0, 0); //HERE
            startTimer();
            return;
        }
    }
    else if (path_ != DRIVE_BACK_DUMB && path_ != NOTHING && path_ != WAIT_5_SECONDS)
    {
        SwervePose *pose = nullptr;
        for (size_t i = pointNum_; i < swervePoints_.size(); ++i)
        {
            // pose = swervePoints_[i].getPose(time, pointOver);
            if (!pathGenerated_ || ((path_ == SECOND_CONE || path_ == SECOND_CUBE_MID || path_ == AUTO_DOCK || (path_ == FIRST_CONE_DOCK && pointNum_ == 1) || (path_ == FIRST_CUBE_DOCK && pointNum_ == 1) || (path_ == SECOND_CONE_DOCK && pointNum_ == 0) || (path_ == SECOND_CUBE_DOCK && pointNum_ == 0)) && (!curveSecondStageGenerated_ || !yawStageGenerated_)))
            {
                SwervePose currPose(swerveDrive_->getX(), swerveDrive_->getY(), swerveDrive_->getYaw(), 0);
                if (path_ == SECOND_CONE || path_ == SECOND_CUBE_MID) // COULDO make the if logic not completely terrible
                {
                    if (!pathGenerated_)
                    {
                        double setX = swervePoints_[i].getX();
                        xTraj_.generateTrajectory(currPose.getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();
                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        if (pointNum_ == 0)
                        {
                            double setYaw = swervePoints_[i].getYaw();
                            yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); // TODO yaw vel
                            yawStageGenerated_ = true;
                        }
                    }

                    if ((!curveSecondStageGenerated_) && (pointNum_ == 1 || (pointNum_ == 0 && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 2)))
                    {
                        double setY = swervePoints_[i].getY();
                        yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;
                    }

                    if (curveSecondStageGenerated_ && !yawStageGenerated_ && pointNum_ == 1)
                    {
                        tuple<double, double, double> yProfile = yTraj_.getProfile();
                        if (get<0>(yProfile) == 0 && get<1>(yProfile) == 0)
                        {
                            double setYaw = swervePoints_[i].getYaw();
                            yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); // TODO yaw vel
                            yawStageGenerated_ = true;
                        }
                    }
                }
                else if (path_ == AUTO_DOCK)
                {
                    if (!pathGenerated_)
                    {
                        double setY = swervePoints_[i].getY();
                        yTraj_.generateTrajectory(currPose.getY(), setY, swerveDrive_->getXYVel().second);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        double setYaw = swervePoints_[i].getYaw();
                        yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); // TODO yaw vel

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.2)
                    {
                        double setX = swervePoints_[i].getX();
                        xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageGenerated_ = true;
                    }
                }
                else if ((path_ == FIRST_CONE_DOCK || path_ == FIRST_CUBE_DOCK) && pointNum_ == 1)
                {
                    if (!pathGenerated_)
                    {
                        double setY = swervePoints_[i].getY();
                        yTraj_.generateTrajectory(currPose.getY(), setY, swerveDrive_->getXYVel().second);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        double setYaw = swervePoints_[i].getYaw();
                        yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); // TODO yaw vel

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1)
                    {
                        double setX = swervePoints_[i].getX();
                        xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageGenerated_ = true;
                    }
                }
                else if ((path_ == SECOND_CONE_DOCK || path_ == SECOND_CUBE_DOCK) && pointNum_ == 0)
                {
                    if (!pathGenerated_)
                    {
                        double setX = swervePoints_[i].getX();
                        xTraj_.generateTrajectory(currPose.getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        double setYaw = swervePoints_[i].getYaw();
                        yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); // TODO yaw vel

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    // frc::SmartDashboard::PutBoolean("GEN", curveSecondStageGenerated_);
                    // frc::SmartDashboard::PutBoolean("PATH", pathGenerated_);
                    // frc::SmartDashboard::PutNumber("TIME", timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_);
                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 2.3)
                    {
                        // frc::SmartDashboard::PutNumber("F", 1);
                        double setY = swervePoints_[i].getY();
                        yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;
                    }
                }
                else
                {
                    currPath_.reset();
                    currPath_.addPoint(currPose);
                    currPath_.addPoint(swervePoints_[i]);

                    currPath_.generateTrajectory(false);

                    pathGenerated_ = true;
                }
            }

            if (path_ == SECOND_CONE || path_ == SECOND_CUBE_MID)
            {
                tuple<double, double, double> xProfile = xTraj_.getProfile();
                tuple<double, double, double> yProfile = {0, 0, swerveDrive_->getY()};
                tuple<double, double, double> yawProfile = {0, 0, swerveDrive_->getYaw()};
                if (curveSecondStageGenerated_)
                {
                    yProfile = yTraj_.getProfile();
                    // frc::SmartDashboard::PutNumber("WY", get<2>(yProfile));
                    // frc::SmartDashboard::PutNumber("WYV", get<1>(yProfile));
                    // frc::SmartDashboard::PutNumber("WYA", get<0>(yProfile));
                    // frc::SmartDashboard::PutNumber("YVel", swerveDrive_->getXYVel().second);
                }
                if (yawStageGenerated_)
                {
                    yawProfile = yawTraj_.getProfile();
                }
                pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));

                if (get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
                {
                    pointOver = true;
                }
            }
            else if (path_ == AUTO_DOCK)
            {
                tuple<double, double, double> xProfile = {0, 0, swerveDrive_->getX()};
                tuple<double, double, double> yProfile = yTraj_.getProfile();
                tuple<double, double, double> yawProfile = yawTraj_.getProfile();
                if (curveSecondStageGenerated_)
                {
                    xProfile = xTraj_.getProfile();
                }
                pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));

                if (get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
                {
                    pointOver = true;
                }
            }
            else if ((path_ == SECOND_CONE_DOCK && pointNum_ == 0) || (path_ == SECOND_CUBE_DOCK && pointNum_ == 0))
            {
                tuple<double, double, double> xProfile = xTraj_.getProfile();
                tuple<double, double, double> yProfile = {0, 0, swerveDrive_->getY()};
                tuple<double, double, double> yawProfile = yawTraj_.getProfile();
                if (curveSecondStageGenerated_)
                {
                    yProfile = yTraj_.getProfile();
                }
                pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));

                if (get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
                {
                    pointOver = true;
                }
            }
            else if ((path_ == FIRST_CONE_DOCK && pointNum_ == 1) || (path_ == FIRST_CUBE_DOCK && pointNum_ == 1))
            {
                tuple<double, double, double> xProfile = {0, 0, swerveDrive_->getX()};
                tuple<double, double, double> yProfile = yTraj_.getProfile();
                tuple<double, double, double> yawProfile = yawTraj_.getProfile();
                if (curveSecondStageGenerated_)
                {
                    xProfile = xTraj_.getProfile();
                }
                pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));

                if (get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
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

        if (pose != nullptr)
        {
            swerveDrive_->drivePose(*pose);
            delete pose;
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

        if (arm_->getPosition() == TwoJointArmProfiles::MID && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
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
            wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
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
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            // wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2)
            {
                cubeIntaking_ = false;
                forward_ = true;
                if (arm_->isForward())
                {
                    if (arm_->getPosition() != TwoJointArmProfiles::CUBE_HIGH)
                    {
                        armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
                    }
                }
            }
            else
            {
                forward_ = false;
                clawOpen_ = true;
                cubeIntaking_ = true;
            }

            if (arm_->getPosition() == TwoJointArmProfiles::CUBE_HIGH && arm_->getState() == TwoJointArm::HOLDING_POS && pointOver)
            {
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
                if (!placingTimerStarted_)
                {
                    placingStartTime_ = timer_.GetFPGATimestamp().value();
                    placingTimerStarted_ = true;
                }

                if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.25)
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
            // wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2)
            {
                cubeIntaking_ = false;
                forward_ = false;
            }

            if (pointOver)
            {
                double ang = (yaw_)*pi / 180.0;                                                     // Radians
                double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    tilt = 0.0;
                }
                double output = -SwerveConstants::AUTOKTILT * tilt;
                swerveDrive_->drive(output, 0, 0);
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
            // wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.5)
            {
                cubeIntaking_ = false;
                forward_ = true;
                if (arm_->isForward())
                {
                    if (arm_->getPosition() != TwoJointArmProfiles::CUBE_MID)
                    {
                        armPosition_ = TwoJointArmProfiles::CUBE_MID;
                    }
                }
            }

            if (arm_->getPosition() == TwoJointArmProfiles::CUBE_MID && arm_->getState() == TwoJointArm::HOLDING_POS && pointOver)
            {
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
                if (!placingTimerStarted_)
                {
                    placingStartTime_ = timer_.GetFPGATimestamp().value();
                    placingTimerStarted_ = true;
                }

                if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.25)
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
            if (pointOver)
            {
                nextPointReady_ = true;
            }
        }
        else
        {
            // wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.5)
            {
                cubeIntaking_ = false;
                forward_ = false;
                armPosition_ = TwoJointArmProfiles::STOWED;
            }

            if (pointOver)
            {
                double ang = (yaw_)*pi / 180.0;                                                     // Radians
                double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    tilt = 0.0;
                }
                double output = -SwerveConstants::AUTOKTILT * tilt;
                swerveDrive_->drive(output, 0, 0);
            }
        }

        break;
    }
    case AUTO_DOCK:
    {
        armPosition_ = TwoJointArmProfiles::STOWED;
        clawOpen_ = false;
        wheelSpeed_ = 0;
        if (pointOver)
        {
            double ang = (yaw_)*pi / 180.0;                                                     // Radians
            double pitch = Helpers::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
            double roll = Helpers::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
            double tilt = pitch * sin(ang) - roll * cos(ang);
            if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
            {
                tilt = 0.0;
            }
            double output = -SwerveConstants::AUTOKTILT * tilt;
            swerveDrive_->drive(output, 0, 0);
        }
        break;
    }
    case NOTHING:
    {
        swerveDrive_->drive(0, 0, 0);
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
        swerveDrive_->drive(0, 0, 0);
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

pair<double, double> AutoPaths::initPos()
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
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
                y = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::TOP_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
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
            x = FieldConstants::BLUE_AUTO_DOCK_X;
        }
        else
        {
            x = FieldConstants::RED_AUTO_DOCK_X;
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