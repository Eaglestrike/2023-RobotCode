#include "SwerveDrive.h"

SwerveDrive::SwerveDrive()
{
    // autoX_ = 0;
    // autoY_ = 0;
    trackingTag_ = false;
    foundTag_ = false;
    tagPos_ = -1;
    tagFollowingStartTime_ = 0;

    // aprilTagX_ = 0;
    // aprilTagY_ = 0;
}

void SwerveDrive::setYaw(double yaw)
{
    yaw_ = yaw;
}

void SwerveDrive::periodic(double yaw, Controls *controls)
{
    frc::SmartDashboard::PutBoolean("Found Tag", foundTag_);

    setYaw(yaw);
    //updateAprilTagFieldXY(); TODO here
    calcOdometry();

    if (frc::DriverStation::IsTeleop())
    {
        if (controls->lXTriggerPressed()) // TODO lJoyTriggerPressed
        {
            if (!trackingTag_)
            {
                tagPos_ = checkScoringPos();

                double wantedX, wantedY, wantedYaw;
                if (tagPos_ < 0 || tagPos_ > 10)
                {
                    drive(controls->getXStrafe(), controls->getYStrafe(), controls->getTurn());
                    return;
                }
                else if (tagPos_ != 10)
                {
                    trackingTag_ = true;
                    wantedY = tagPos_ * 0.5588;
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        wantedYaw = 90;
                        wantedX = FieldConstants::BLUE_SCORING_X;
                    }
                    else
                    {
                        wantedYaw = -90;
                        wantedX = FieldConstants::RED_SCORING_X;
                    }
                }
                else
                {
                    trackingTag_ = true;
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
                    {
                        wantedYaw = -90;
                        wantedX = 16.178784 - 0.5;
                        wantedY = 6.749796;
                    }
                    else
                    {
                        wantedYaw = -90;
                        wantedX = 0.36195 + 0.5;
                        wantedY = 6.749796;
                    }
                }
                SwervePose currPose(robotX_, robotY_, yaw_, 0, 0, 0, 0, 0, 0);        // TODO vel and acc, use odometry to start?
                SwervePose wantedPose(wantedX, wantedY, wantedYaw, 0, 0, 0, 0, 0, 0); // TODO vel and acc, figure out yaw somehow (arm or closest)

                tagPath_.reset();
                tagPath_.addPoint(currPose);
                tagPath_.addPoint(wantedPose);
                tagPath_.generateTrajectory(false);
                tagFollowingStartTime_ = timer_.GetFPGATimestamp().value();

                // TEST BELOW
            }

            bool end = false;
            double time = timer_.GetFPGATimestamp().value() - tagFollowingStartTime_;
            SwervePose *wantedPose = tagPath_.getPose(time, end);
            if (!end)
            {
                // drivePose(*wantedPose);
            }
            delete wantedPose;
        }
        else
        {
            trackingTag_ = false;
            double xStrafe, yStrafe;
            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
            {
                xStrafe = controls->getYStrafe();
                yStrafe = -controls->getXStrafe();
            }
            else
            {
                xStrafe = -controls->getYStrafe();
                yStrafe = controls->getXStrafe();
            }
            drive(xStrafe, yStrafe, controls->getTurn());
        }
    }
}

void SwerveDrive::drive(double xSpeed, double ySpeed, double turn)
{
    calcModules(xSpeed, ySpeed, turn, false);

    // double volts = frc::SmartDashboard::GetNumber("Swerve Volts", 0.0);

    // if(abs(xSpeed) > 0.1 || abs(ySpeed) > 0.1 || abs(turn) > 0.1)
    // {
    //     topRight_->periodic(volts, trAngle_, true);
    //     topLeft_->periodic(volts, tlAngle_, true);
    //     bottomRight_->periodic(volts, brAngle_, true);
    //     bottomLeft_->periodic(volts, blAngle_, true);
    // }
    // else
    // {
    //     topRight_->periodic(0, trAngle_, true);
    //     topLeft_->periodic(0, tlAngle_, true);
    //     bottomRight_->periodic(0, brAngle_, true);
    //     bottomLeft_->periodic(0, blAngle_, true);
    // }

    // 0.65, 0, 0
    // 1, 610, 0.15534
    // 2, 2480, 0.63156
    // 3, 4600, 1.17144
    // 4, 6300, 1.60436
    // 5, 8400, 2.13915
    // 6, 10200, 2.59754
    // m = 2.044, b = 0.669435

    // turn
    // 1, 530, 9.66532
    // 2, 2400, 43.7675
    // 3, 4000, 72.94584
    // 4, 6200, 113.0661
    // 5, 8100, 147.7153
    // 6, 9800, 178.7173
    // m = 0.0291946, b = 0.746574

    topRight_->periodic(trSpeed_, trAngle_, false);
    topLeft_->periodic(tlSpeed_, tlAngle_, false);
    bottomRight_->periodic(brSpeed_, brAngle_, false);
    bottomLeft_->periodic(blSpeed_, blAngle_, false);

    // double speed = frc::SmartDashboard::GetNumber("Swerve Volts", 0);
    // topRight_->periodic(speed, trAngle_, true);
    // topLeft_->periodic(speed, tlAngle_, true);
    // bottomRight_->periodic(speed, brAngle_, true);
    // bottomLeft_->periodic(speed, blAngle_, true);
}

/*void SwerveDrive::driveAutoPose(double yaw, SwervePose pose)
{
    setYaw(yaw);
    calcOdometry();

    /*frc::SmartDashboard::PutNumber("AX", autoX_);
    frc::SmartDashboard::PutNumber("AY", autoY_);
    frc::SmartDashboard::PutNumber("WAX", pose.getX());
    frc::SmartDashboard::PutNumber("WAY", pose.getY());

    frc::SmartDashboard::PutNumber("WVX", pose.getXVel());
    frc::SmartDashboard::PutNumber("WVY", pose.getYVel());*

    double xVel = pose.getXVel();
    double yVel = pose.getYVel();
    if(pose.getXVel() != 0 || pose.getYVel() != 0)
    {
        xVel += (pose.getX() - autoX_) * SwerveConstants::klP + pose.getXAcc() * SwerveConstants::klA;
        yVel += (pose.getY() - autoY_) * SwerveConstants::klP + pose.getYAcc() * SwerveConstants::klA;
    }


    double yawVel = pose.getYawVel();
    //frc::SmartDashboard::PutNumber("WYAW", yawVel);

    //frc::SmartDashboard::PutNumber("wy", pose.getYaw());
    if(pose.getXVel() != 0 || pose.getYVel() != 0 || pose.getYawVel() != 0)
    {
        yawVel += (pose.getYaw() - (-yaw_)) * SwerveConstants::kaP + pose.getYawAcc() * SwerveConstants::kaA;
    }

    calcModules(xVel, yVel, yawVel, true);

    topRight_->periodic(trSpeed_, trAngle_, true);
    topLeft_->periodic(tlSpeed_, tlAngle_, true);
    bottomRight_->periodic(brSpeed_, brAngle_, true);
    bottomLeft_->periodic(blSpeed_, blAngle_, true);
}*/

void SwerveDrive::drivePose(SwervePose pose)
{
    double xVel = pose.getXVel();
    double yVel = pose.getYVel();

    if (pose.getXVel() != 0 || pose.getYVel() != 0)
    {
        yVel += (pose.getX() - robotX_) * SwerveConstants::klP + pose.getXAcc() * SwerveConstants::klA;
        xVel += (pose.getY() - robotY_) * SwerveConstants::klP + pose.getYAcc() * SwerveConstants::klA;
    }

    double yawVel = pose.getYawVel();

    if (pose.getXVel() != 0 || pose.getYVel() != 0 || pose.getYawVel() != 0)
    {
        yawVel += (pose.getYaw() - (-yaw_)) * SwerveConstants::kaP + pose.getYawAcc() * SwerveConstants::kaA;
    }

    calcModules(xVel, yVel, yawVel, true);

    topRight_->periodic(trSpeed_, trAngle_, true);
    topLeft_->periodic(tlSpeed_, tlAngle_, true);
    bottomRight_->periodic(brSpeed_, brAngle_, true);
    bottomLeft_->periodic(blSpeed_, blAngle_, true);
}

void SwerveDrive::calcModules(double xSpeed, double ySpeed, double turn, bool inVolts)
{
    double angle = yaw_ * (pi / 180);

    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    if (inVolts)
    {
        turn = (turn * pi / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2);
    }

    double turnComponent = sqrt(turn * turn / 2);
    if (turn < 0)
    {
        turnComponent *= -1;
    }

    double A = newX - (turnComponent);
    double B = newX + (turnComponent);
    double C = newY - (turnComponent);
    double D = newY + (turnComponent);

    trSpeed_ = sqrt(B * B + C * C);
    tlSpeed_ = sqrt(B * B + D * D);
    brSpeed_ = sqrt(A * A + C * C);
    blSpeed_ = sqrt(A * A + D * D);

    if (xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        trAngle_ = -atan2(B, C) * 180 / pi;
        tlAngle_ = -atan2(B, D) * 180 / pi;
        brAngle_ = -atan2(A, C) * 180 / pi;
        blAngle_ = -atan2(A, D) * 180 / pi;
    }

    double maxSpeed;
    if (inVolts)
    {
        if (trSpeed_ != 0)
        {
            trSpeed_ = (trSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if (tlSpeed_ != 0)
        {
            tlSpeed_ = (tlSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if (brSpeed_ != 0)
        {
            brSpeed_ = (brSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        if (blSpeed_ != 0)
        {
            blSpeed_ = (blSpeed_ - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        maxSpeed = GeneralConstants::MAX_VOLTAGE;
    }
    else
    {
        maxSpeed = 1;
    }

    if (trSpeed_ > maxSpeed || tlSpeed_ > maxSpeed || brSpeed_ > maxSpeed || brSpeed_ > maxSpeed)
    {
        double max = trSpeed_;

        max = (tlSpeed_ > max) ? tlSpeed_ : max;
        max = (brSpeed_ > max) ? brSpeed_ : max;
        max = (blSpeed_ > max) ? blSpeed_ : max;

        trSpeed_ = (trSpeed_ / max);
        tlSpeed_ = (tlSpeed_ / max);
        brSpeed_ = (brSpeed_ / max);
        blSpeed_ = (blSpeed_ / max);

        if (inVolts)
        {
            trSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            tlSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            brSpeed_ *= GeneralConstants::MAX_VOLTAGE;
            blSpeed_ *= GeneralConstants::MAX_VOLTAGE;
        }
    }
}

void SwerveDrive::calcOdometry()
{
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    if (!frc::DriverStation::IsEnabled())
    {
        reset();
        return;
    }

    pair<double, double> xyVel = getXYVel();

    // if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    // {
    //     robotX_ += rotatedY * dT_; //Changed here on purpose to switch cords cause the field isn't mirrored and I hate it
    //     robotY_ -= rotatedX * dT_;
    // }
    // else
    // {
    //     robotX_ -= rotatedY * dT_; //Changed here on purpose to switch cords cause the field isn't mirrored and I hate it
    //     robotY_ += rotatedX * dT_;
    // }

    robotX_ += xyVel.first * dT_;
    robotY_ += xyVel.second * dT_;

    frc::SmartDashboard::PutNumber("x", robotX_);
    frc::SmartDashboard::PutNumber("y", robotY_);
}

void SwerveDrive::reset()
{
    robotX_ = 0;
    robotY_ = 0;
    foundTag_ = false;
}

double SwerveDrive::getX()
{
    return robotX_;
}

double SwerveDrive::getY()
{
    return robotY_;
}

pair<double, double> SwerveDrive::getXYVel()
{
    double frX = -topRight_->getDriveVelocity() * sin(topRight_->getAngle() * pi / 180);
    double frY = topRight_->getDriveVelocity() * cos(topRight_->getAngle() * pi / 180);
    double flX = -topLeft_->getDriveVelocity() * sin(topLeft_->getAngle() * pi / 180);
    double flY = topLeft_->getDriveVelocity() * cos(topLeft_->getAngle() * pi / 180);
    double brX = -bottomRight_->getDriveVelocity() * sin(bottomRight_->getAngle() * pi / 180);
    double brY = bottomRight_->getDriveVelocity() * cos(bottomRight_->getAngle() * pi / 180);
    double blX = -bottomLeft_->getDriveVelocity() * sin(bottomLeft_->getAngle() * pi / 180);
    double blY = bottomLeft_->getDriveVelocity() * cos(bottomLeft_->getAngle() * pi / 180);

    double avgX = (frX + flX + brX + blX) / 4;
    double avgY = (frY + flY + brY + blY) / 4;

    double angle = yaw_ * pi / 180;

    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    return {rotatedX, rotatedY};
}

double SwerveDrive::getYaw()
{
    return yaw_;
}

void SwerveDrive::setPos(pair<double, double> xy)
{
    robotX_ = xy.first;
    robotY_ = xy.second;
}

void SwerveDrive::updateAprilTagFieldXY()
{
    double defaultVal[] = {-1};
    vector<double> data = frc::SmartDashboard::GetNumberArray("data", defaultVal);

    if (data.at(0) == -1)
    {
        foundTag_ = false;
        return;
    }

    double tagX = data.at(2);
    double tagY = data.at(3);
    double tagZAng = -data.at(7);
    int tagID = data.at(1);

    // frc::SmartDashboard::PutNumber("Tag x", tagX);
    // frc::SmartDashboard::PutNumber("Tag y", tagY);
    // frc::SmartDashboard::PutNumber("Tag ZAng", tagZAng);

    double orientedTagX = tagX * cos(tagZAng) + tagY * sin(tagZAng);
    double orientedTagY = tagX * -sin(tagZAng) + tagY * cos(tagZAng);

    // TODO check for bad id
    if (tagID < 1 || tagID > 8)
    {
        return;
    }

    double fieldTagX = FieldConstants::TAG_XY[tagID - 1][0];
    double fieldTagY = FieldConstants::TAG_XY[tagID - 1][1];

    // frc::SmartDashboard::PutNumber("GX", fieldTagX);
    // frc::SmartDashboard::PutNumber("GY", fieldTagY);

    // frc::SmartDashboard::PutNumber("TOX", orientedTagX);
    // frc::SmartDashboard::PutNumber("TOY", orientedTagY);

    double aprilTagX, aprilTagY;
    aprilTagY = fieldTagY + orientedTagX;
    if (tagID == 1 || tagID == 2 || tagID == 3 || tagID == 4)
    {
        aprilTagX = fieldTagX - orientedTagY;
    }
    else
    {
        aprilTagX = fieldTagX + orientedTagY;
    }

    // frc::SmartDashboard::PutNumber("Field X", aprilTagX);
    // frc::SmartDashboard::PutNumber("Field y", aprilTagY);

    if (!foundTag_) // TODO check if necessary
    {
        robotX_ = aprilTagX;
        robotY_ = aprilTagY;
        foundTag_ = true;
    }
    else
    {
        robotX_ += (-robotX_ + aprilTagX) * 0.05;
        robotY_ += (-robotY_ + aprilTagY) * 0.05;
    }
}

int SwerveDrive::checkScoringPos() // TODO get better values
{
    if (!foundTag_)
    {
        return -1;
    }

    if (robotX_ > 2.919857 && robotX_ < 13.621893)
    {
        return -1;
    }

    if (robotX_ < 0 || robotY_ < 0 || robotX_ > 16.54175 || robotY_ > 8.0137)
    {
        return -1;
    }

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
    {
        if (robotX_ >= 13.621893)
        {
            if (robotY_ > 5.487416)
            {
                return 10;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (robotY_ > 5.487426975)
            {
                return -1;
            }
            else
            {
                int scoringPos = (int)round(robotY_ / 0.54864);
                if (scoringPos == 10)
                {
                    scoringPos = 9; // TODO fix, yeah yeah
                }
                return scoringPos;
            }
        }
    }
    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed)
    {
        if (robotX_ <= 2.919857)
        {
            if (robotY_ > 5.487416)
            {
                return 10;
            }
            else
            {
                return -1;
            }
        }
        else
        {
            if (robotY_ > 5.487426975)
            {
                return -1;
            }
            else
            {
                int scoringPos = (int)round(robotY_ / 0.54864);
                if (scoringPos == 10)
                {
                    scoringPos = 9; // TODO fix, yeah yeah
                }
                return scoringPos;
            }
        }
    }

    return -1;
}