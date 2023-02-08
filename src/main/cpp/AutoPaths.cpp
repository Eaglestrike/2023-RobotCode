#include "AutoPaths.h"

AutoPaths::AutoPaths(SwerveDrive *swerveDrive) : swerveDrive_(swerveDrive)
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
    case PRELOADED_CONE:
    {
        // swervePoints_.push_back(SwervePose(0, 0, 0, 0));
        break;
    }
    case PRELOADED_CUBE:
    {
        // swervePoints_.push_back(SwervePose(0, 0, 0, 0));
        break;
    }
    case FIRST_CONE:
    {
        // double deltaX = 0.4064;
        // if (mirrored_)
        // {
        //     deltaX *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        // swervePoints_.push_back(SwervePose(0, 0, 0, 0));

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
                y2 = FieldConstants::TOP_CONE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y;
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
                y2 = FieldConstants::TOP_CONE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case FIRST_CUBE:
    {
        // double deltaX = 0.4064;
        // if (mirrored_)
        // {
        //     deltaX *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));

        // double returnX = 0.555752;
        // if (mirrored_)
        // {
        //     returnX *= -1;
        // }
        // swervePoints_.push_back(SwervePose(0, returnX, 0, 0));
        // break;

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
                y2 = FieldConstants::TOP_CUBE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X + 3; //HERE
            x2 = FieldConstants::RED_SCORING_X;
            yaw1 = 90;
            yaw2 = 90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case SECOND_CONE:
    {
        // double deltaX = 0.4064;
        // if (mirrored_)
        // {
        //     deltaX *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        // double deltaX2 = 1.2192;
        // double yaw = 90;
        // if (mirrored_)
        // {
        //     deltaX2 *= -1;
        //     yaw *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX+deltaX2, 5.2451, yaw, 0.8));
        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0.8));
        // swervePoints_.push_back(SwervePose(0, 0, 0, 0));

        //------------------------------------------------------

        // double x1, x2, xe, y1, y2, ye, yaw1, yaw2, yawe;
        // if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        // {
        //     x1 = FieldConstants::BLUE_PIECE_X;
        //     x2 = FieldConstants::BLUE_SCORING_X;
        //     xe = FieldConstants::BLUE_PIECE_X;
        //     yaw1 = -90;
        //     yaw2 = -90;
        //     if (mirrored_)
        //     {
        //         y1 = FieldConstants::TOP_PIECE_Y;
        //         y2 = FieldConstants::TOP_CONE_Y;
        //         ye = FieldConstants::TOP_MID_PIECE_Y;
        //         yawe = 180;
        //     }
        //     else
        //     {
        //         y1 = FieldConstants::BOTTOM_PIECE_Y;
        //         y2 = FieldConstants::BOTTOM_CONE_Y;
        //         ye = FieldConstants::BOTTOM_MID_PIECE_Y;
        //         yawe = 0;
        //     }
        // }
        // else
        // {
        //     x1 = FieldConstants::RED_PIECE_X;
        //     x2 = FieldConstants::RED_SCORING_X;
        //     xe = FieldConstants::RED_PIECE_X;
        //     yaw1 = 90;
        //     yaw2 = 90;
        //     if (!mirrored_)
        //     {
        //         y1 = FieldConstants::TOP_PIECE_Y;
        //         y2 = FieldConstants::TOP_CONE_Y;
        //         ye = FieldConstants::TOP_MID_PIECE_Y;
        //         yawe = 180;
        //     }
        //     else
        //     {
        //         y1 = FieldConstants::BOTTOM_PIECE_Y;
        //         y2 = FieldConstants::BOTTOM_CONE_Y;
        //         xe = FieldConstants::BOTTOM_MID_PIECE_Y;
        //         yawe = 0;
        //     }
        // }

        // swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        // swervePoints_.push_back(SwervePose(xe, ye, yawe, 0));
        // swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        // swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw1 = -45;
            yaw2 = -90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CONE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y;
            }
        }
        else
        {
            x1 = FieldConstants::RED_PIECE_X + 3; //HERE
            x2 = FieldConstants::RED_SCORING_X;
            yaw1 = 90;
            yaw2 = 90;
            if (!mirrored_)
            {
                y1 = FieldConstants::TOP_MID_PIECE_Y+0.3; //HERE
                y2 = FieldConstants::TOP_CONE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CONE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE:
    {
        // double deltaX = 0.4064;
        // if (mirrored_)
        // {
        //     deltaX *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0));
        // double deltaX2 = 1.2192;
        // double yaw = 90;
        // if (mirrored_)
        // {
        //     deltaX2 *= -1;
        //     yaw *= -1;
        // }

        // swervePoints_.push_back(SwervePose(deltaX+deltaX2, 5.2451, yaw, 0.8));
        // swervePoints_.push_back(SwervePose(deltaX, 5.2451, 0, 0.8));

        // double returnX = 0.555752;
        // if (mirrored_)
        // {
        //     returnX *= -1;
        // }
        // swervePoints_.push_back(SwervePose(0, returnX, 0, 0));

        //------------------------------------------------------

        // double x1, x2, xe, y1, y2, ye, yaw1, yaw2, yawe;
        // if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        // {
        //     x1 = FieldConstants::BLUE_PIECE_X;
        //     x2 = FieldConstants::BLUE_SCORING_X;
        //     xe = FieldConstants::BLUE_PIECE_X;
        //     yaw1 = -90;
        //     yaw2 = -90;
        //     if (mirrored_)
        //     {
        //         y1 = FieldConstants::TOP_PIECE_Y;
        //         y2 = FieldConstants::TOP_CUBE_Y;
        //         ye = FieldConstants::TOP_MID_PIECE_Y;
        //         yawe = 180;
        //     }
        //     else
        //     {
        //         y1 = FieldConstants::BOTTOM_PIECE_Y;
        //         y2 = FieldConstants::BOTTOM_CUBE_Y;
        //         ye = FieldConstants::BOTTOM_MID_PIECE_Y;
        //         yawe = 0;
        //     }
        // }
        // else
        // {
        //     x1 = FieldConstants::RED_PIECE_X;
        //     x2 = FieldConstants::RED_SCORING_X;
        //     xe = FieldConstants::RED_PIECE_X;
        //     yaw1 = 90;
        //     yaw2 = 90;
        //     if (!mirrored_)
        //     {
        //         y1 = FieldConstants::TOP_PIECE_Y;
        //         y2 = FieldConstants::TOP_CUBE_Y;
        //         ye = FieldConstants::TOP_MID_PIECE_Y;
        //         yawe = 180;
        //     }
        //     else
        //     {
        //         y1 = FieldConstants::BOTTOM_PIECE_Y;
        //         y2 = FieldConstants::BOTTOM_CUBE_Y;
        //         xe = FieldConstants::BOTTOM_MID_PIECE_Y;
        //         yawe = 0;
        //     }
        // }

        // swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        // swervePoints_.push_back(SwervePose(xe, ye, yawe, 0.8));
        // swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        // swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        double x1, x2, y1, y2, yaw1, yaw2;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x1 = FieldConstants::BLUE_PIECE_X;
            x2 = FieldConstants::BLUE_SCORING_X;
            yaw1 = -45;
            yaw2 = -90;
            if (mirrored_)
            {
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y;
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
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::TOP_CUBE_Y;
            }
            else
            {
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::BOTTOM_CUBE_Y;
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
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            x = FieldConstants::BLUE_AUTO_DOCK_X;
            yaw = -90;
        }
        else
        {
            x = FieldConstants::RED_AUTO_DOCK_X;
            yaw = 90;
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

void AutoPaths::setActions(Path a1, Path a2, Path a3)
{
    actions_.clear();
    actions_.push_back(a1);
    actions_.push_back(a2);
    actions_.push_back(a3);

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

void AutoPaths::periodic(SwerveDrive *swerveDrive)
{
    if (!actionsSet_)
    {
        return;
    }

    if (!pathSet_)
    {
        if (actionNum_ > 2)
        {
            swerveDrive->drive(0, 0, 0);
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
    if (path_ == PRELOADED_CONE || path_ == PRELOADED_CUBE)
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
    else if (path_ != DRIVE_BACK_DUMB && path_ != NOTHING)
    {
        SwervePose *pose = nullptr;
        for (size_t i = pointNum_; i < swervePoints_.size(); ++i)
        {
            // pose = swervePoints_[i].getPose(time, pointOver);
            if (!pathGenerated_ || ((path_ == SECOND_CONE || path_ == SECOND_CUBE) && !curveSecondStageGenerated_))
            {
                SwervePose currPose(swerveDrive_->getX(), swerveDrive_->getY(), swerveDrive_->getYaw(), 0);
                if (path_ == SECOND_CONE || path_ == SECOND_CUBE) //COULDO make the if logic not completely terrible
                {
                    if(!pathGenerated_)
                    {
                        double setX = swervePoints_[i].getX();
                        xTraj_.generateTrajectory(currPose.getX(), setX, swerveDrive_->getXYVel().first);
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();
                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        if(pointNum_ == 0)
                        {
                            double setYaw = swervePoints_[i].getYaw();
                            yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); //TODO yaw vel
                            yawStageGenerated_ = true;
                        }
                    }
                    
                    if ((!curveSecondStageGenerated_) && (pointNum_ == 1 || (pointNum_ == 0 && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1)))
                    {
                        double setY = swervePoints_[i].getY();
                        yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().second);
                        curveSecondStageGenerated_ = true;
                    }

                    if(curveSecondStageGenerated_ && !yawStageGenerated_ && pointNum_ == 1)
                    {
                        tuple<double, double, double> yProfile = yTraj_.getProfile();
                        if(get<0>(yProfile) == 0 && get<1>(yProfile) == 0)
                        {
                            double setYaw = swervePoints_[i].getYaw();
                            yawTraj_.generateTrajectory(currPose.getYaw(), setYaw, 0); //TODO yaw vel
                            yawStageGenerated_ = true;
                        }
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

            if (path_ == SECOND_CONE || path_ == SECOND_CUBE)
            {
                tuple<double, double, double> xProfile = xTraj_.getProfile();
                tuple<double, double, double> yProfile = {0, 0, swerveDrive_->getY(),};
                tuple<double, double, double> yawProfile = {0, 0, swerveDrive_->getYaw()};
                if(curveSecondStageGenerated_)
                {
                    yProfile = yTraj_.getProfile();
                    // frc::SmartDashboard::PutNumber("WY", get<2>(yProfile));
                    // frc::SmartDashboard::PutNumber("WYV", get<1>(yProfile));
                    // frc::SmartDashboard::PutNumber("WYA", get<0>(yProfile));
                    // frc::SmartDashboard::PutNumber("YVel", swerveDrive_->getXYVel().second);
                }
                if(yawStageGenerated_)
                {
                    //yawProfile = yawTraj_.getProfile();
                }
                pose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));
                
                
                if(get<0>(xProfile) == 0 && get<1>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yawProfile) == 0)
                {
                    pointOver = true;
                }
            }
            else
            {
                pose = currPath_.getPose(time, pointOver);
            }

            if (!pointOver)
            {
                break;
            }
            else
            {
                if (nextPointReady_ && i == swervePoints_.size() - 1)
                {
                    // frc::SmartDashboard::PutBoolean("f", true);
                    pathSet_ = false;
                    nextPointReady_ = false;
                    ++actionNum_;
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
                    // frc::SmartDashboard::PutNumber("what", 1);
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
            swerveDrive->drivePose(*pose);
            delete pose;
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
    case PRELOADED_CONE:
    {
        // TODO place cone
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case PRELOADED_CUBE:
    {
        // TODO place cones
        pointOver = true;
        nextPointReady_ = true;
        break;
    }
    case FIRST_CONE:
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
    case FIRST_CUBE:
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
    case SECOND_CONE:
    {
        // TODO see if curves are the same
        if (pointOver && pointNum_ != 3)
        {
            nextPointReady_ = true;
        }

        // TODO place cone
        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 3)
        {
            nextPointReady_ = true;
        }
        break;
    }
    case SECOND_CUBE:
    {
        // TODO see if curves are the same
        if (pointOver && pointNum_ != 3)
        {
            nextPointReady_ = true;
        }

        // TODO place cone
        // if done placing
        if (/*done placing &&*/ pointOver && pointNum_ == 3)
        {
            nextPointReady_ = true;
        }
        break;
    }
    case AUTO_DOCK:
    {
        if (pointOver)
        {
            // TODO auto dock
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
        if (!failsafeStarted_)
        {
            failsafeStarted_ = true;
            failsafeTimer_.Stop();
            failsafeTimer_.Reset();
            failsafeTimer_.Start();
        }

        if (failsafeTimer_.Get().value() < 2)
        {
            swerveDrive_->drive(0, 0.2, 0);
        }
        else
        {
            swerveDrive_->drive(0, 0, 0);
        }

        break;
    }
    }
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
        return 90;
    }
    else
    {
        return -90;
    }
}

pair<double, double> AutoPaths::initPos()
{
    switch (actions_[0])
    {
    case PRELOADED_CONE:
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
                y = FieldConstants::TOP_CONE_Y;
            }
            else
            {
                y = FieldConstants::BOTTOM_CONE_Y;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CONE_Y;
            }
            else
            {
                y = FieldConstants::TOP_CONE_Y;
            }
        }

        return {x, y};
    }
    case PRELOADED_CUBE:
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
                y = FieldConstants::TOP_CUBE_Y;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y;
            }
        }
        else
        {
            if (mirrored_)
            {
                y = FieldConstants::BOTTOM_CUBE_Y;
            }
            else
            {
                y = FieldConstants::TOP_CUBE_Y;
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