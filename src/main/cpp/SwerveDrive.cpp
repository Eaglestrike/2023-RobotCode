#include "SwerveDrive.h"

/*
* Constructor
*/
SwerveDrive::SwerveDrive()
{
    // autoX_ = 0;
    // autoY_ = 0;
    trackingTag_ = false;
    foundTag_ = false;
    tagPos_ = -1;
    tagFollowingStartTime_ = 0;
    prevTag_ = -1;

    // aprilTagX_ = 0;
    // aprilTagY_ = 0;
}

/*
* Setter for yaw, i.e. the angle of the robot
*
* @param yaw yaw to set
*/
void SwerveDrive::setYaw(double yaw)
{
    yaw_ = yaw;
}

/*
* Periodic function called, setting new yaw from navX and controller pointer
* @param yaw yaw from navX
* @param controls controls pointer
*/
void SwerveDrive::periodic(double yaw, Controls *controls)
{
    frc::SmartDashboard::PutBoolean("Found Tag", foundTag_);

    setYaw(yaw);
    updateAprilTagFieldXY();
    calcOdometry();

    if (frc::DriverStation::IsTeleop()) // If teleop (might be better in a separate TeleopPeriodic function)
    {
        if (controls->lJoyTriggerPressed())
        {
            if (!trackingTag_)
            {
                tagPos_ = checkScoringPos();
                frc::SmartDashboard::PutNumber("Tag Pos", tagPos_);

                double wantedX, wantedY, wantedYaw;
                if (tagPos_ < 0 || tagPos_ > 10)
                {
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
                        wantedYaw = 90;
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
                // SwervePose currPose(robotX_, robotY_, yaw_, 0);        // TODO vel and acc, use odometry to start?
                // SwervePose wantedPose(wantedX, wantedY, wantedYaw, 0); // TODO vel and acc, figure out yaw somehow (arm or closest)

                // tagPath_.reset();
                // tagPath_.addPoint(currPose);
                // tagPath_.addPoint(wantedPose);
                // tagPath_.generateTrajectory(false);
                // tagFollowingStartTime_ = timer_.GetFPGATimestamp().value();

                // TEST BELOW FOR WHILE MOVING

                xTagTraj_.generateTrajectory(robotX_, wantedX, (getXYVel().first));
                yTagTraj_.generateTrajectory(robotY_, wantedY, (getXYVel().second));
                yawTagTraj_.generateTrajectory(yaw_, wantedYaw, 0);

                frc::SmartDashboard::PutNumber("STARTX", robotX_);
                frc::SmartDashboard::PutNumber("STARTY", robotY_);
            }

            // bool end = false;
            // double time = timer_.GetFPGATimestamp().value() - tagFollowingStartTime_;
            // frc::SmartDashboard::PutNumber("T", time);
            // SwervePose *wantedPose = tagPath_.getPose(time, end);
            // frc::SmartDashboard::PutNumber("WX", wantedPose->getX());
            // frc::SmartDashboard::PutNumber("WY", wantedPose->getY());
            // frc::SmartDashboard::PutNumber("WYAW", wantedPose->getYaw());

            // if (!end)
            // {
            //     frc::SmartDashboard::PutBoolean("Driving", true);
            //     drivePose(*wantedPose);
            // }
            // else
            // {
            //     frc::SmartDashboard::PutBoolean("Driving", false);
            //     adjustPos(SwervePose(wantedPose->getX(), wantedPose->getY(), wantedPose->getYaw(), 0, 0, 0, 0, 0, 0));
            // }
            // delete wantedPose;

            tuple<double, double, double> xProfile = xTagTraj_.getProfile();
            tuple<double, double, double> yProfile = yTagTraj_.getProfile();
            tuple<double, double, double> yawProfile = yawTagTraj_.getProfile();

            SwervePose *wantedPose = new SwervePose(get<2>(xProfile), get<2>(yProfile), get<2>(yawProfile), get<1>(xProfile), get<1>(yProfile), get<1>(yawProfile), get<0>(xProfile), get<0>(yProfile), get<0>(yawProfile));
            frc::SmartDashboard::PutNumber("WX", wantedPose->getX());
            frc::SmartDashboard::PutNumber("WY", wantedPose->getY());
            frc::SmartDashboard::PutNumber("WYAW", wantedPose->getYaw());
            drivePose(*wantedPose);
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

/*
* Drives the robot at the speed and angle
*/
void SwerveDrive::drive(double xSpeed, double ySpeed, double turn)
{
    calcModules(xSpeed, ySpeed, 0, 0, turn, 0, false);

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
    double yawVel = pose.getYawVel();

    if (/*pose.getXVel() != 0 || pose.getYVel() != 0 || pose.getYawVel() != 0*/ true)
    {
        if (pose.getXVel() == 0 && pose.getXAcc() == 0)
        {
            // adjust x because x path done
            double xError = (pose.getX() - robotX_);
            if (abs(xError) < 0.03)
            {
                xVel = 0;
            }
            else
            {
                xVel = (pose.getX() - robotX_) * SwerveConstants::klP * 6;
            }
        }
        else
        {
            // normal x stuff and path still going
            xVel += (pose.getX() - robotX_) * SwerveConstants::klP + (pose.getXVel() - getXYVel().first) * SwerveConstants::klD;
        }

        if (pose.getYVel() == 0 && pose.getYAcc() == 0)
        {
            // adjust y because y path done
            double yError = (pose.getY() - robotY_);
            if (abs(yError) < 0.03)
            {
                yVel = 0;
            }
            else
            {
                yVel = (pose.getY() - robotY_) * SwerveConstants::klP * 6;
            }
        }
        else
        {
            // normal y stuff and path still going
            yVel += (pose.getY() - robotY_) * SwerveConstants::klP + (pose.getYVel() - getXYVel().second) * SwerveConstants::klD;
        }

        if (pose.getYawVel() == 0 && pose.getYawAcc() == 0)
        {
            // adjust yaw because yaw path done
            double yawVel;
            double yawError = (pose.getYaw() - yaw_);
            if (abs(yawError) < 1)
            {
                yawVel = 0;
            }
            else
            {
                yawVel = (pose.getYaw() - (yaw_)) * SwerveConstants::kaP * 8;
            }
        }
        else
        {
            // normal yaw stuff and path still going
            yawVel += (pose.getYaw() - (yaw_)) * SwerveConstants::kaP;
        }
    }

    // frc::SmartDashboard::PutNumber("XE", pose.getX() - robotX_);
    // frc::SmartDashboard::PutNumber("YE", pose.getY() - robotY_);
    // frc::SmartDashboard::PutNumber("XVE", pose.getXVel() - getXYVel().first);
    // frc::SmartDashboard::PutNumber("YVE", pose.getYVel() - getXYVel().second);

    // frc::SmartDashboard::PutNumber("XVel", xVel);
    // frc::SmartDashboard::PutNumber("YVel", yVel);
    // frc::SmartDashboard::PutNumber("YawVel", -yawVel);
    calcModules(xVel, yVel, pose.getXAcc(), pose.getYAcc(), -yawVel, -pose.getYawAcc(), true);
    // calcModules(xVel, yVel, 0, 0, -yawVel, 0, true);

    topRight_->periodic(trSpeed_, trAngle_, true);
    topLeft_->periodic(tlSpeed_, tlAngle_, true);
    bottomRight_->periodic(brSpeed_, brAngle_, true);
    bottomLeft_->periodic(blSpeed_, blAngle_, true);
}

void SwerveDrive::adjustPos(SwervePose pose)
{
    double xVel;
    double xError = (pose.getX() - robotX_);
    if (abs(xError) < 0.03)
    {
        xVel = 0;
    }
    else
    {
        xVel = (pose.getX() - robotX_) * SwerveConstants::klP * 6;
    }

    double yVel;
    double yError = (pose.getY() - robotY_);
    if (abs(yError) < 0.03)
    {
        yVel = 0;
    }
    else
    {
        yVel = (pose.getY() - robotY_) * SwerveConstants::klP * 6;
    }

    double yawVel;
    double yawError = (pose.getYaw() - yaw_);
    if (abs(yawError) < 1)
    {
        yawVel = 0;
    }
    else
    {
        yawVel = (pose.getYaw() - (yaw_)) * SwerveConstants::kaP * 8;
    }

    // calcModules(xVel, yVel, pose.getXAcc(), pose.getYAcc(), -yawVel, -pose.getYawAcc(), true);
    calcModules(xVel, yVel, 0, 0, -yawVel, 0, true);

    topRight_->periodic(trSpeed_, trAngle_, true);
    topLeft_->periodic(tlSpeed_, tlAngle_, true);
    bottomRight_->periodic(brSpeed_, brAngle_, true);
    bottomLeft_->periodic(blSpeed_, blAngle_, true);
}


/*
* Calculates the module's orientation and power output
*/
void SwerveDrive::calcModules(double xSpeed, double ySpeed, double xAcc, double yAcc, double turn, double turnAcc, bool inVolts)
{
    //https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf
    //https://www.desmos.com/calculator/bxk1qdap5l incomplete desmos
    //Robot's angle in radians
    double angle = yaw_ * (pi / 180);

    //Rotate the velocities by the robot's rotation (Robot-Orient the velocity) i.e. decompose vectors 
    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    //Rotate acceleration (Robot-Orient)
    double newXAcc = xAcc * cos(angle) + yAcc * sin(angle);
    double newYAcc = yAcc * cos(angle) + xAcc * -sin(angle);

    //Calculate the turning if the turn and turnaccel is in degrees
    if (inVolts)
    {
        //Convert to radians, then multiply by the radius to get tangential speed & accelerations
        turn = (turn * pi / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2);
        turnAcc = (turnAcc * pi / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2);
    }

    //Scale the velocity and acceleration
    double turnComponent = sqrt(turn * turn / 2);
    double turnAccComponent = sqrt(turnAcc * turnAcc / 2);
    if (turn < 0)
    {
        turnComponent *= -1;
    }
    if (turnAcc < 0)
    {
        turnAccComponent *= -1;
    }

    //Math
    double A = newX - (turnComponent);
    double B = newX + (turnComponent);
    double C = newY - (turnComponent);
    double D = newY + (turnComponent);

    double trVel = sqrt(B * B + C * C);
    double tlVel = sqrt(B * B + D * D);
    double brVel = sqrt(A * A + C * C);
    double blVel = sqrt(A * A + D * D);

    double AA = newXAcc - (turnAccComponent);
    double BA = newXAcc + (turnAccComponent);
    double CA = newYAcc - (turnAccComponent);
    double DA = newYAcc + (turnAccComponent);

    double trAcc = sqrt(BA * BA + CA * CA);
    double tlAcc = sqrt(BA * BA + DA * DA);
    double brAcc = sqrt(AA * AA + CA * CA);
    double blAcc = sqrt(AA * AA + DA * DA);

    if (xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        trAngle_ = -atan2(B, C) * 180 / pi; // TODO check B + BA
        tlAngle_ = -atan2(B, D) * 180 / pi;
        brAngle_ = -atan2(A, C) * 180 / pi;
        blAngle_ = -atan2(A, D) * 180 / pi;
    }

    double maxSpeed;
    if (inVolts)
    {
        if (trVel != 0)
        {
            trSpeed_ = /*(trAcc / SwerveConstants::klA) + */ (trVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }
        else
        {
            trSpeed_ = 0;
        }

        if (tlVel != 0)
        {
            tlSpeed_ = /*(tlAcc / SwerveConstants::klA) + */ (tlVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }
        else
        {
            tlSpeed_ = 0;
        }

        if (brVel != 0)
        {
            brSpeed_ = /*(brAcc / SwerveConstants::klA) + */ (brVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }
        else
        {
            brSpeed_ = 0;
        }

        if (blVel != 0)
        {
            blSpeed_ = /*(blAcc / SwerveConstants::klA) + */ (blVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }
        else
        {
            blSpeed_ = 0;
        }

        maxSpeed = GeneralConstants::MAX_VOLTAGE;
    }
    else
    {
        trSpeed_ = trVel;
        tlSpeed_ = tlVel;
        brSpeed_ = brVel;
        blSpeed_ = blVel;
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

/**
 * Updates odometry based off wheel readings
*/
void SwerveDrive::calcOdometry()
{
    //Find difference in time between frames
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    if (!frc::DriverStation::IsEnabled())//Do not calc odomotry if the robot is not moving; reset it
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

    //Euler's integration
    robotX_ += xyVel.first * dT_;
    robotY_ += xyVel.second * dT_;

    frc::SmartDashboard::PutNumber("x", robotX_);
    frc::SmartDashboard::PutNumber("y", robotY_);
}

/**
 * Resets robot position and information
*/
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

/**
 * Returns pair of xy velocities
*/
pair<double, double> SwerveDrive::getXYVel()
{
    //TODO don't call the same method twice four times
    //Get robot oriented x,y velocities of each swerve module
    double frX = -topRight_->getDriveVelocity() * sin(topRight_->getAngle() * pi / 180);
    double frY = topRight_->getDriveVelocity() * cos(topRight_->getAngle() * pi / 180);
    double flX = -topLeft_->getDriveVelocity() * sin(topLeft_->getAngle() * pi / 180);
    double flY = topLeft_->getDriveVelocity() * cos(topLeft_->getAngle() * pi / 180);
    double brX = -bottomRight_->getDriveVelocity() * sin(bottomRight_->getAngle() * pi / 180);
    double brY = bottomRight_->getDriveVelocity() * cos(bottomRight_->getAngle() * pi / 180);
    double blX = -bottomLeft_->getDriveVelocity() * sin(bottomLeft_->getAngle() * pi / 180);
    double blY = bottomLeft_->getDriveVelocity() * cos(bottomLeft_->getAngle() * pi / 180);

    //Average robot-oriented velocties
    double avgX = (frX + flX + brX + blX) / 4;
    double avgY = (frY + flY + brY + blY) / 4;

    // cout << timer_.GetFPGATimestamp().value() << ", " << sqrt(avgX * avgX + avgY * avgY) << endl;

    //Angle in radians
    double angle = yaw_ * pi / 180;

    //Unrotate the velocties to field-oriented
    double rotatedX = avgX * cos(angle) + avgY * -sin(angle);
    double rotatedY = avgX * sin(angle) + avgY * cos(angle);

    return {rotatedX, rotatedY};
}


/**
 * Getter for yaw
 * @returns rotation of the robot
*/
double SwerveDrive::getYaw()
{
    return yaw_;
}

/*
* Setter for position
* @param xy
*/
void SwerveDrive::setPos(pair<double, double> xy)
{
    robotX_ = xy.first;
    robotY_ = xy.second;
}

/**
 * Using april tags to update field odometry
*/
void SwerveDrive::updateAprilTagFieldXY()
{
    //-1 is no april tag
    double defaultVal[] = {-1};
    //Get data from SmartDashboard/Networktables
    vector<double> data = frc::SmartDashboard::GetNumberArray("data", defaultVal);

    if (data.at(0) == -1)//No april tag data
    {
        foundTag_ = false;
        return;
    }
    //Format of 
    //[ apriltag or nah , id , x , y , _ , _ , _ , zang]
    double tagX = data.at(2);
    double tagY = data.at(3);
    double tagZAng = -data.at(7);
    int tagID = data.at(1);

    // frc::SmartDashboard::PutNumber("Tag x", tagX);
    // frc::SmartDashboard::PutNumber("Tag y", tagY);
    // frc::SmartDashboard::PutNumber("Tag ZAng", tagZAng);

    //Rotate the tag by the seen angle to orient displacement to field orient
    double orientedTagX = tagX * cos(tagZAng) + tagY * sin(tagZAng);
    double orientedTagY = tagX * -sin(tagZAng) + tagY * cos(tagZAng);

    if (tagID == -1)//Wrong tag id
    {
        if(robotX_ > 3.919857 && robotX_ < 12.621893) //If robot is not near community nor loading station
        {
            foundTag_ = false;
        }
        return;
    }
    if (tagID < 1 || tagID > 8) //Ignore for now
    {
        return;
    }

    if (tagID != prevTag_) //If tag changes
    {
        prevTag_ = tagID; //Set to change, do nothing (skip a frame)
        return;
    }
    prevTag_ = tagID; //Set the past known tag to be this tag

    //Get xy of field tag position
    double fieldTagX = FieldConstants::TAG_XY[tagID - 1][0]; 
    double fieldTagY = FieldConstants::TAG_XY[tagID - 1][1];

    // frc::SmartDashboard::PutNumber("GX", fieldTagX);
    // frc::SmartDashboard::PutNumber("GY", fieldTagY);

    // frc::SmartDashboard::PutNumber("TOX", orientedTagX);
    // frc::SmartDashboard::PutNumber("TOY", orientedTagY);


    double aprilTagX, aprilTagY;
    //Really just field-oriented coordinates (idk why called apriltagY) - call SeenPositionX or smthg
    //Adding the displacement from tag to robot + apriltag position
    aprilTagY = fieldTagY + orientedTagX;
    if (tagID == 1 || tagID == 2 || tagID == 3 || tagID == 4)//Left side of field
    {
        aprilTagX = fieldTagX - orientedTagY;
    }
    else//Right side of field
    {
        aprilTagX = fieldTagX + orientedTagY;
    }

    // frc::SmartDashboard::PutNumber("Field X", aprilTagX);
    // frc::SmartDashboard::PutNumber("Field y", aprilTagY);

    //1st check = set it regardless
    if (!foundTag_) // TODO check if necessary
    {
        robotX_ = aprilTagX;
        robotY_ = aprilTagY;
        foundTag_ = true;
    }
    else
    {
        //Update by moving the position to the calculated position if the position is close
        //Increment it proportionally by the error
        if (abs(robotX_ - aprilTagX) < 1 && abs(robotY_ - aprilTagY) < 1) // TODO watch out here
        {
            robotX_ += (-robotX_ + aprilTagX) * 0.05;
            robotY_ += (-robotY_ + aprilTagY) * 0.05;
        }
    }
}

/**
 * Checks if the position is scorable
 * @returns tag id idk help? -1 is no position
*/
int SwerveDrive::checkScoringPos() // TODO get better values
{
    if (!foundTag_) // If tag not found
    {
        return -1;
    }

    if (robotX_ > 3.919857 && robotX_ < 12.621893) //If robot is not near community nor loading station
    {
        return -1;
    }

    if (robotX_ < 0 || robotY_ < 0 || robotX_ > FieldConstants::FIELD_LENGTH || robotY_ > FieldConstants::FIELD_WIDTH) //If rbot is out of bounds
    {
        return -1;
    }

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)//Blue alliance bum boom bum bum boom
    {
        if (robotX_ >= FieldConstants::TAG_XY[0][0]) //TODO make parameters better for all TAG_XY, probably make it check y first
        {
            if (robotY_ > FieldConstants::TAG_XY[2][1])
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
            if (robotY_ > FieldConstants::TAG_XY[2][1])
            {
                return -1;
            }
            else if(robotX_ > 3.919857)
            {
                return -1;
            }
            else
            {
                //int scoringPos = (int)round(robotY_ / 0.5588);
                int scoringPos = (int)round((robotY_ - (0.5*FieldConstants::TAG_XY[0][1])) / 0.5588);
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
        if (robotX_ <= FieldConstants::TAG_XY[5][0]) //TODO make parameters beter for all TAG_XY, probably make it check y first
        {
            if (robotY_ > FieldConstants::TAG_XY[2][1])
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
            if (robotY_ > FieldConstants::TAG_XY[2][1])
            {
                return -1;
            }
            else if(robotX_ < 12.621893)
            {
                return -1;
            }
            else
            {
                int scoringPos = (int)round(robotY_ / 0.5588);
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