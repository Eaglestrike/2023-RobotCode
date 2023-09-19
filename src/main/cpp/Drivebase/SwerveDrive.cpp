#include "Drivebase/SwerveDrive.h"

#include <math.h>

#include "Helpers/GeometryHelper.h"
#include "Helpers/GeneralPoses.h"
using namespace Poses;

/*
 * Constructor
 */
SwerveDrive::SwerveDrive():
    trackingTag_(false),
    trackingPlayerStation_(false),
    foundTag_(false),
    setTagPos_(1),
    tagFollowingStartTime_(0),
    prevTag_(-1),
    prevUniqueVal_(-1),
    holdingYaw_(0),
    isHoldingYaw_(false),
    LineupTrim_(0,0),
    numLargeDiffs_(0)
{
    // autoX_ = 0;
    // autoY_ = 0;
    // inching_ = false;

    // aprilTagX_ = 0;
    // aprilTagY_ = 0;

    // yawTagOffset_ = 0;
}


/// @brief Updates swerve drive with data collected or given (should be called in normal periodic)
/// @param yaw rotation around vertical axis
/// @param tilt rotation around world x axis
/// @param data camera data
void SwerveDrive::periodic(double yaw, double tilt, std::vector<double> data)
{
    config_.isBlue = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    yaw_ = yaw;
    calcOdometry();
    updateAprilTagFieldXY(tilt, data);

    topRight_->periodic();
    topLeft_->periodic();
    bottomRight_->periodic();
    bottomLeft_->periodic();
}

/// @brief Trims the robot's alignment in the direction give. Shifts odometry
/// @param xTrimDirection the trim x in inches
/// @param yTrimDirection the trim y in inches
void SwerveDrive::trim(double xTrimDirection, double yTrimDirection){
    Vector addTrim{
        xTrimDirection*SwerveConstants::TRIMMING_DIST,
        yTrimDirection*SwerveConstants::TRIMMING_DIST
    };

    if(config_.isBlue){
        LineupTrim_ += addTrim.rotateClockwise90();
    }
    else{
        LineupTrim_ += addTrim.rotateCounterclockwise90();
    }

    frc::SmartDashboard::PutNumber("X Trim", LineupTrim_.getX() / 0.0254);
    frc::SmartDashboard::PutNumber("Y Trim", LineupTrim_.getY() / 0.0254);
}

/// @brief Basically if the POV is pressed of the slow button is pressed, either will just engage in a slow control
/// @param inchUp bool
/// @param inchDown bool
/// @param inchLeft bool
/// @param inchRight bool
/// @param slow bool - toggle
void SwerveDrive::inch(double inchUp, double inchDown, double inchLeft, double inchRight, double slow){
    config_.isSlow = slow;
    if(inchUp || inchDown || inchLeft || inchRight){
        config_.isSlow = true;
    }
}

void SwerveDrive::teleopPeriodic(bool score, bool forward, int scoringLevel, bool islockWheels, bool autoBalance)
{
    // frc::SmartDashboard::PutBoolean("Found Tag", foundTag_);
    frc::SmartDashboard::PutNumber("STime", timer_.GetFPGATimestamp().value());

    frc::SmartDashboard::PutBoolean("Scoring", score);
    if (score)
    {
        if (!trackingTag_)
        {
            holdingYaw_ = false;
            Point scoringPos = checkScoringPos(scoringLevel);
            // frc::SmartDashboard::PutNumber("SX", scoringPos.first);
            // frc::SmartDashboard::PutNumber("SY", scoringPos.second);
            if (scoringPos.isZero()) // Did not find scoring pos: COULDO get a better flag thing
            {
                drive(strafe_, rotation_);
                return;
            }
            autoLineup(scoringPos); //Generates trajectory
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

        //Follow trajectory
        const Pose1D xProfile = xTagTraj_.getProfile();
        const Pose1D yProfile = yTagTraj_.getProfile();
        const Pose1D yawProfile = yawTagTraj_.getProfile();

        if (trackingPlayerStation_)
        {
            //Reset Trajectory if at target
            // if (get<0>(yProfile) == 0 && get<0>(yawProfile) == 0 && get<1>(yProfile) == 0 && get<1>(yawProfile) == 0)
            // {
            //     if (abs(robotY_ - get<2>(yProfile)) > 0.0254 * 4)
            //     {
            //         trackingTag_ = false;
            //         trackingPlayerStation_ = false;
            //         return;
            //     }
            // }

            //Follow trajectory
            // double xStrafe;
            // if (config_.isBlue)
            // {
            //     xStrafe = strafe_.getY() * SwerveConstants::MAX_TELE_VEL;
            // }
            // else
            // {
            //     xStrafe = -strafe_.getY() * SwerveConstants::MAX_TELE_VEL;
            // }
            // SwervePose wantedPose = SwerveFromPose1D({getX(), xStrafe, 0}, yProfile, yawProfile);
            // drivePose(wantedPose);

            drive(strafe_, rotation_)
        }
        else
        {
            if (isStationary(xProfile) && isStationary(yProfile) && isStationary(yawProfile)) //Not moving
            {
                if (abs(robotX_ - xProfile.pos) > 0.08 || abs(robotY_ - yProfile.pos) > 0.08) // Not at profile when finished
                {
                    trackingTag_ = false; //Reset scoring
                    trackingPlayerStation_ = false;
                    return;
                }
            }

            SwervePose wantedPose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
            // frc::SmartDashboard::PutNumber("WX", wantedPose.x);
            // frc::SmartDashboard::PutNumber("WY", wantedPose.y);
            // frc::SmartDashboard::PutNumber("WYAW", wantedPose.yaw);
            drivePose(wantedPose);
        }
    }
    else if (islockWheels)
    {
        lockWheels();
    }
    else if (!autoBalance)
    {
        trackingTag_ = false;
        trackingPlayerStation_ = false;
        Vector fieldStrafe;
        if (config_.isBlue)
        {
            fieldStrafe = strafe_.rotateClockwise90();
        }
        else
        {
            fieldStrafe = strafe_.rotateCounterclockwise90();
        }
        double turn = rotation_;

        drive(strafe_, turn);

        // if (abs(xStrafe) < 0.005 && abs(yStrafe) < 0.005 && abs(turn) < 0.02)
        // {
        //     if (!inching_)
        //     {
        //         if (inchUp)
        //         {
        //             inching_ = true;
        //             Point vel = getXYVel();
        //             if (config_.isBlue)
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_ + SwerveConstants::INCHING_DIST, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_, vel.second);
        //             }
        //             else
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_ - SwerveConstants::INCHING_DIST, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_, vel.second);
        //             }
        //         }
        //         else if (inchDown)
        //         {
        //             inching_ = true;
        //             Point vel = getXYVel();
        //             if (config_.isBlue)
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_ - SwerveConstants::INCHING_DIST, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_, vel.second);
        //             }
        //             else
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_ + SwerveConstants::INCHING_DIST, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_, vel.second);
        //             }
        //         }
        //         else if (inchLeft)
        //         {
        //             inching_ = true;
        //             Point vel = getXYVel();
        //             if (config_.isBlue)
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_ + SwerveConstants::INCHING_DIST, vel.second);
        //             }
        //             else
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_ - SwerveConstants::INCHING_DIST, vel.second);
        //             }
        //         }
        //         else if (inchRight)
        //         {
        //             inching_ = true;
        //             Point vel = getXYVel();
        //             if (config_.isBlue)
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_ - SwerveConstants::INCHING_DIST, vel.second);
        //             }
        //             else
        //             {
        //                 xTagTraj_.generateTrajectory(robotX_, robotX_, vel.first);
        //                 yTagTraj_.generateTrajectory(robotY_, robotY_ + SwerveConstants::INCHING_DIST, vel.second);
        //             }
        //         }
        //         else
        //         {
        //             drive(0, 0, 0);
        //         }
        //     }
        //     else
        //     {
        //         tuple<double, double, double> xProfile = xTagTraj_.getProfile();
        //         tuple<double, double, double> yProfile = yTagTraj_.getProfile();
        //         SwervePose wantedPose(get<2>(xProfile), get<2>(yProfile), yaw_, get<1>(xProfile), get<1>(yProfile), 0, get<0>(xProfile), get<0>(yProfile), 0);
        //         drivePose(wantedPose);

        //         if (get<0>(xProfile) == 0 && get<0>(yProfile) == 0 && get<1>(xProfile) == 0 && get<1>(yProfile) == 0)
        //         {
        //             inching_ = false;
        //         }
        //     }
        // }
        // else
        // {
        //     inching_ = false;
        //     drive(xStrafe, yStrafe, turn);
        // }
    }
}

/// @brief Sets the target 
/// @param xStrafe [-1.0, 1.0]
/// @param yStrafe [-1.0, 1.0]
/// @param rotation [-1.0, 1.0]
void SwerveDrive::setTarget(double xStrafe, double yStrafe, double rotation){
    rotation_ = rotation / 4.0;
    strafe_ = {xStrafe, yStrafe};
    if (config_.isBlue){
        strafe_.rotateClockwise90This();
    }
    else{
        strafe_.rotateCounterclockwise90This();
    }
    if(config_.isSlow){ //Reduce speed of robot
        double vel = strafe_.getMagnitude();
        double newVel = vel;
        if (vel != 0)
        {
            newVel *= 0.09;
            newVel += 0.06;
            strafe_ *= newVel/vel;
        }

        rotation_ *= 0.3; // 0.15
        if (abs(rotation_) < 0.3 * 0.3 * 0.2)
        {
            rotation_ = 0;
        }
        // if (rotation_ > 0)
        // {
        //     rotation_ += 0.05; // 0.05
        // }
        // else if (rotation_ < 0)
        // {
        //     rotation_ -= 0.05;
        // }
    }
    else if(config_.isPanic){
        rotation_ = std::clamp(rotation_, -0.075, 0.075);
    }
}

/// @brief if the arm is out or not, do not swing arm like sling
/// @param panic if to slow down rotation or not
void SwerveDrive::setPanic(bool panic){
    config_.isPanic = panic;
}

/// @brief Sets up the target and trajectory
/// @param scoringPos position on xy world coordinates
void SwerveDrive::autoLineup(Point scoringPos){
    double wantedX = scoringPos.getX();
    double wantedY = scoringPos.getY();
    double wantedYaw;
    bool playerStation; //At the playerstation

    // Code to flip robot if arm is the wrong way
    // if (config_.isBlue)
    // {
    //     if (forward)
    //     {
    //         wantedYaw = 90.0;
    //         if (scoringPos.first > 6.0)
    //         {
    //             wantedY += SwerveConstants::CLAW_MID_OFFSET;
    //             wantedYaw *= -1;
    //             playerStation = true;
    //         }
    //         else
    //         {
    //             wantedY -= SwerveConstants::CLAW_MID_OFFSET;
    //         }
    //     }
    //     else
    //     {
    //         wantedYaw = -90.0;
    //         if (scoringPos.first > 6.0)
    //         {
    //             wantedYaw *= -1;
    //             wantedY -= SwerveConstants::CLAW_MID_OFFSET;
    //             playerStation = true;
    //         }
    //         else
    //         {
    //             wantedY += SwerveConstants::CLAW_MID_OFFSET;
    //         }
    //     }
    // }
    // else
    // {
    //     if (forward)
    //     {
    //         wantedYaw = -90.0;
    //         if (scoringPos.first < 6.0)
    //         {
    //             wantedY -= SwerveConstants::CLAW_MID_OFFSET;
    //             wantedYaw *= -1;
    //             playerStation = true;
    //         }
    //         else
    //         {
    //             wantedY += SwerveConstants::CLAW_MID_OFFSET;
    //         }
    //     }
    //     else
    //     {
    //         wantedYaw = 90.0;
    //         if (scoringPos.first < 6.0)
    //         {
    //             wantedY += SwerveConstants::CLAW_MID_OFFSET;
    //             wantedYaw *= -1;
    //             playerStation = true;
    //         }
    //         else
    //         {
    //             wantedY -= SwerveConstants::CLAW_MID_OFFSET;
    //         }
    //     }
    // }

    if (yaw_ > 0)
    {
        wantedYaw = 90;
    }
    else
    {
        wantedYaw = -90;
    }

    //TODO: test if this is needed
    if (setTagPos_ == 9)
    {
        wantedYaw += 5;
    }

    playerStation = FieldConstants::onPlayerStationHalf(config_.isBlue, scoringPos.getX());

    wantedY += (wantedYaw > 0) ? -SwerveConstants::CLAW_MID_OFFSET : SwerveConstants::CLAW_MID_OFFSET;
    trackingPlayerStation_ = playerStation;
    if(trackingPlayerStation_){//Don't generate trajectory
        return;
    }

    // if (playerStation)
    // {
    //     if ((config_.isBlue && getX() > FieldConstants::PLAYER_STATION_X.blue - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][0]) || (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && getX() < FieldConstants::PLAYER_STATION_X.red + TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][0]))
    //     {
    //         double xStrafe, yStrafe;
    //         if (config_.isBlue)
    //         {
    //             xStrafe = controls->getYStrafe();
    //             yStrafe = -controls->getXStrafe();
    //         }
    //         else
    //         {
    //             xStrafe = -controls->getYStrafe();
    //             yStrafe = controls->getXStrafe();
    //         }
    //         double turn = controls->getTurn();
    //         if (panic)
    //         {
    //             turn = std::clamp(turn, -0.075, 0.075);
    //         }
    //         trackingTag_ = false;
    //         drive(xStrafe, yStrafe, turn);
    //         return;
    //     }
    // }

    frc::SmartDashboard::PutNumber("WX", wantedX);
    frc::SmartDashboard::PutNumber("WY", wantedY);
    xTagTraj_.generateTrajectory(robotX_, wantedX, (getXYVel().getX()));
    yTagTraj_.generateTrajectory(robotY_, wantedY, (getXYVel().getY()));
    yawTagTraj_.generateTrajectory(yaw_, wantedYaw, 0);

    trackingTag_ = true;
}

/*
 * Drives the robot at the speed and angle
 */
void SwerveDrive::drive(Vector strafe, double turn)
{
    double xSpeed = strafe_.getX();
    double ySpeed = strafe_.getY();
    if (turn == 0) //Yaw adjustment to prevent drift over time
    {
        double yawError = GeometryHelper::getAngDiffDeg(yaw_, holdingYaw_);
        if (!isHoldingYaw_)
        {
            holdingYaw_ = yaw_;
            isHoldingYaw_ = true;
        }
        else if (abs(yawError) > 5) //Greater than 5 degrees
        {
            holdingYaw_ = yaw_;
        }
        if (abs(yawError) > 0.5 && (abs(xSpeed) > 0.1 || abs(ySpeed) > 0.1))
        {
            turn = -(yawError)*0.02;
        }
    }
    else
    {
        isHoldingYaw_ = false;
    }

    calcModules(xSpeed, ySpeed, /*0, 0,*/ turn, /*0,*/ false);

    // double volts = frc::SmartDashboard::GetNumber("Swerve Volts", 0.0);

    // if(abs(xSpeed) > 0.1 || abs(ySpeed) > 0.1 || abs(turn) > 0.1)
    // {
    //     topRight_->move({volts, trAngle_, true);
    //     topLeft_->move({volts, tlAngle_, true);
    //     bottomRight_->move({volts, brAngle_, true);
    //     bottomLeft_->move({volts, blAngle_, true);
    // }
    // else
    // {
    //     topRight_->move({0, trAngle_, true);
    //     topLeft_->move({0, tlAngle_, true);
    //     bottomRight_->move({0, brAngle_, true);
    //     bottomLeft_->move({0, blAngle_, true);
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

    topRight_->move({trSpeed_, trAngle_}, false);
    topLeft_->move({tlSpeed_, tlAngle_}, false);
    bottomRight_->move({brSpeed_, brAngle_}, false);
    bottomLeft_->move({blSpeed_, blAngle_}, false);

    // double speed = frc::SmartDashboard::GetNumber("Swerve Volts", 0);
    // topRight_->move({speed, trAngle_, true);
    // topLeft_->move({speed, tlAngle_, true);
    // bottomRight_->move({speed, brAngle_, true);
    // bottomLeft_->move({speed, blAngle_, true);
}

void SwerveDrive::lockWheels()
{
    trSpeed_ = 0;
    tlSpeed_ = 0;
    brSpeed_ = 0;
    blSpeed_ = 0;
    trAngle_ = 135;
    tlAngle_ = 45;
    brAngle_ = 45;
    blAngle_ = 135;

    topRight_->move({trSpeed_, trAngle_}, false);
    topLeft_->move({tlSpeed_, tlAngle_}, false);
    bottomRight_->move({brSpeed_, brAngle_}, false);
    bottomLeft_->move({blSpeed_, blAngle_}, false);
}

/// @brief Drives the swerve module following the pose
/// @param pose target pose, feedforward velocities
void SwerveDrive::drivePose(const SwervePose pose)
{
    double xVel = pose.xVel;
    double yVel = pose.yVel;
    double yawVel = pose.yawVel;

    Vector currVel = getXYVel();

    // HERE
    //  setPos(Point{pose.x, pose.y});
    //  setYaw(pose.yaw);

    //Adding PD control to feedforward
    if (isMoving(pose) || frc::DriverStation::IsTeleop()){
        //X fine tuning
        double xError = pose.x - robotX_;
        double vxError = pose.xVel - currVel.getX();
        if(pose.xVel == 0 && pose.xAcc == 0){ //xpath done/ no target movement on x axis
            if(abs(xError) < 0.0254 * 1)//Error less than 1 inch
            { 
                xError = 0.0;
            }
            vxError = 0.0;
        }
        xVel += (xError) * SwerveConstants::klP + (vxError) * SwerveConstants::klD; //additional PD control correction
        
        //Y fine tuning
        double yError = pose.y - robotY_;
        double vyError = pose.yVel - currVel.getY();
        if (pose.yVel == 0 && pose.yAcc == 0){ //ypath done/ no target movement on y axis
            if (abs(yError) < 0.0254 * 1) //Error less than 1 inch
            {
                yError = 0.0;
            }
            vyError = 0.0;
        }
        yVel += (yError) * SwerveConstants::klP + (vyError) * SwerveConstants::klD; //additional PD control correction

        //yaw fine tuning
        double yawError = GeometryHelper::getAngDiffDeg(yaw_, pose.yaw);
        if (pose.yawVel == 0 && pose.yawAcc == 0){ // yaw path done/ no target movement on rotation axis
            if (abs(yawError) < 0.5) //Error less than 0.5 degrees
            {
                yawVel = 0;
            }
            else{
                yawError *= 1.8;
            }
        }
        yawVel += (yawError)*SwerveConstants::kaP; // Just P control correction (wheels can easily stop rotation)
    }

    // frc::SmartDashboard::PutNumber("XE", pose.x - robotX_);
    // frc::SmartDashboard::PutNumber("YE", pose.y - robotY_);
    // frc::SmartDashboard::PutNumber("XVE", pose.xVel - getXYVel().first);
    // frc::SmartDashboard::PutNumber("YVE", pose.yVel - getXYVel().second);

    // frc::SmartDashboard::PutNumber("XVel", xVel);
    // frc::SmartDashboard::PutNumber("YVel", yVel);
    // frc::SmartDashboard::PutNumber("YawVel", -yawVel);
    calcModules(xVel, yVel, /*pose.getXAcc(), pose.yAcc,*/ -yawVel, /*-pose.yawAcc,*/ true);
    // calcModules(xVel, yVel, 0, 0, -yawVel, 0, true);

    topRight_->move({trSpeed_, trAngle_}, true);
    topLeft_->move({tlSpeed_, tlAngle_}, true);
    bottomRight_->move({brSpeed_, brAngle_}, true);
    bottomLeft_->move({blSpeed_, blAngle_}, true);
}

void SwerveDrive::adjustPos(SwervePose pose)
{
    double xVel = 0.0;
    double xError = pose.x - robotX_;
    if (abs(xError) > 0.03){
        xVel = (pose.x - robotX_) * SwerveConstants::klP * 6;
    }

    double yVel = 0.0;
    double yError = pose.y - robotY_;
    if (abs(yError) > 0.03){
        yVel = (pose.y - robotY_) * SwerveConstants::klP * 6;
    }

    double yawVel = 0.0;
    double yawError = GeometryHelper::getAngDiffDeg(yaw_, pose.yaw);
    if (abs(yawError) > 1){
        yawVel = (yawError) * SwerveConstants::kaP * 8;
    }

    // calcModules(xVel, yVel, pose.getXAcc(), pose.yAcc, -yawVel, -pose.yawAcc, true);
    calcModules(xVel, yVel, /*0, 0,*/ -yawVel, /*0,*/ true);

    topRight_->move({trSpeed_, trAngle_}, true);
    topLeft_->move({tlSpeed_, tlAngle_}, true);
    bottomRight_->move({brSpeed_, brAngle_}, true);
    bottomLeft_->move({blSpeed_, blAngle_}, true);
}


/// @brief Calculates the module's orientation and power output
/// @param xSpeed x speed field-oriented
/// @param ySpeed y speed field-oriented
/// @param turn target turn velocity (deg/s if inVolts, rad/s if not inVolts)
/// @param inVolts if the speed should be voltage or [-1, 1]
void SwerveDrive::calcModules(double xSpeed, double ySpeed, /*double xAcc, double yAcc,*/ double turn, /*double turnAcc,*/ bool inVolts)
{
    // https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf
    // https://www.desmos.com/calculator/bxk1qdap5l incomplete desmos
    // Robot's angle in radians
    double angle = yaw_ * (M_PI / 180);

    // Rotate the velocities by the robot's rotation (Robot-Orient the velocity)
    double newX = xSpeed * cos(angle) + ySpeed * sin(angle);
    double newY = ySpeed * cos(angle) + xSpeed * -sin(angle);

    // Rotate acceleration (Robot-Orient)
    // double newXAcc = xAcc * cos(angle) + yAcc * sin(angle);
    // double newYAcc = yAcc * cos(angle) + xAcc * -sin(angle);

    // Calculate the turning if the turn and turnaccel is in degrees
    if (inVolts)
    {
        // Convert to radians, then multiply by the radius to get tangential speed & accelerations
        turn = (turn * M_PI / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2); //(m/s) v = r * w
        // turnAcc = (turnAcc * M_PI / 180) * (SwerveConstants::WHEEL_DIAGONAL / 2);
    }

    // Scale the velocity and acceleration 
    double turnComponent = sqrt(0.5) * turn;

    // https://www.first1684.com/uploads/2/0/1/6/20161347/chimiswerve_whitepaper__2_.pdf
    // https://www.chiefdelphi.com/uploads/default/original/3X/8/c/8c0451987d09519712780ce18ce6755c21a0acc0.pdf
    //Math is just simplified from normal implementation (utilizing symmetry of drivebase)
    double A = newX - (turnComponent);
    double B = newX + (turnComponent);
    double C = newY - (turnComponent);
    double D = newY + (turnComponent);

    double trVel = sqrt(B * B + C * C);
    double tlVel = sqrt(B * B + D * D);
    double brVel = sqrt(A * A + C * C);
    double blVel = sqrt(A * A + D * D);

    // double AA = newXAcc - (turnAccComponent);
    // double BA = newXAcc + (turnAccComponent);
    // double CA = newYAcc - (turnAccComponent);
    // double DA = newYAcc + (turnAccComponent);

    // double trAcc = sqrt(BA * BA + CA * CA);
    // double tlAcc = sqrt(BA * BA + DA * DA);
    // double brAcc = sqrt(AA * AA + CA * CA);
    // double blAcc = sqrt(AA * AA + DA * DA);

    //Prevent swerve modules from returning to 0 when stationary
    if (xSpeed != 0 || ySpeed != 0 || turn != 0)
    {
        //Set target module angles
        trAngle_ = -atan2(B, C) * 180 / M_PI;
        tlAngle_ = -atan2(B, D) * 180 / M_PI;
        brAngle_ = -atan2(A, C) * 180 / M_PI;
        blAngle_ = -atan2(A, D) * 180 / M_PI;
    }
    // else
    // {
    //     trAngle_ = 90;
    //     tlAngle_ = 90;
    //     brAngle_ = 90;
    //     blAngle_ = 90;
    // }

    double maxSpeed;
    if (inVolts)
    {
        trSpeed_ = 0;
        if (trVel != 0){
            trSpeed_ = /*(trAcc / SwerveConstants::klA) + */ (trVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        tlSpeed_ = 0;
        if (tlVel != 0)
        {
            tlSpeed_ = /*(tlAcc / SwerveConstants::klA) + */ (tlVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        brSpeed_ = 0;
        if (brVel != 0)
        {
            brSpeed_ = /*(brAcc / SwerveConstants::klA) + */ (brVel - SwerveConstants::klVI) / SwerveConstants::klV;
        }

        blSpeed_ = 0;
        if (blVel != 0)
        {
            blSpeed_ = /*(blAcc / SwerveConstants::klA) + */ (blVel - SwerveConstants::klVI) / SwerveConstants::klV;
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
    // Find difference in time between frames
    double time = timer_.GetFPGATimestamp().value();
    dT_ = time - prevTime_;
    prevTime_ = time;

    // if (!frc::DriverStation::IsEnabled()) // Do not calc odomotry if the robot is not moving; reset it
    // {
    //     reset();
    //     return;
    // }

    Vector xyVel = getXYVel();

    // if(config_.isBlue)
    // {
    //     robotX_ += rotatedY * dT_; //Changed here on purpose to switch cords cause the field isn't mirrored and I hate it
    //     robotY_ -= rotatedX * dT_;
    // }
    // else
    // {
    //     robotX_ -= rotatedY * dT_; //Changed here on purpose to switch cords cause the field isn't mirrored and I hate it
    //     robotY_ += rotatedX * dT_;
    // }

    // Euler's integration
    robotX_ += xyVel.getX() * dT_;
    robotY_ += xyVel.getY() * dT_;

    if (foundTag_)
    {
        Point xy = {robotX_, robotY_};
        std::pair<Point, Vector> pose{xy, xyVel};

        prevPoses_.insert(std::pair<double, std::pair<Point, Vector>>{time, pose});

        std::map<double, std::pair<Point, Vector>>::iterator it;
        for (it = prevPoses_.begin(); it->first < (time - SwerveConstants::POSE_HISTORY_LENGTH); prevPoses_.erase(it++));

        // frc::SmartDashboard::PutNumber("Prev Pose Count", prevPoses_.size());
    }
}

/**
 * Resets robot position and information
 */
void SwerveDrive::reset()
{
    // robotX_ = 0;
    // robotY_ = 0;
    // foundTag_ = false;
    // prevPoses_.clear();
    isHoldingYaw_ = false;
    // inching_ = false;
    LineupTrim_ = {0,0};
    numLargeDiffs_ = 0;
    // resetYawTagOffset();
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
 * Returns xy velocities
 */
Vector SwerveDrive::getXYVel()
{
    // Get robot oriented x,y velocities of each swerve module
    Vector frV = topRight_->getVelocity();
    Vector flV = topLeft_->getVelocity();
    Vector brV = bottomRight_->getVelocity();
    Vector blV = bottomLeft_->getVelocity();

    // Average robot-oriented velocties
    Vector avgV = (frV + flV + brV + blV) / 4;

    //Rotate to be oriented to front of robot minus the navx
    avgV.rotateCounterclockwise90This();

    // cout << timer_.GetFPGATimestamp().value() << ", " << sqrt(avgX * avgX + avgY * avgY) << endl;

    // Angle in radians
    double angle = yaw_ * M_PI / 180;

    // Unrotate the velocties to field-oriented
    avgV.rotateThis(angle);

    return avgV;
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
void SwerveDrive::setPos(Point xy)
{
    robotX_ = xy.getX();
    robotY_ = xy.getY();
}

/**
 * Using april tags to update field odometry
 */
void SwerveDrive::updateAprilTagFieldXY(double tilt, std::vector<double> data)
{
    //-1 is no april tag
    // double defaultVal[] = {-1};
    // Get data from SmartDashboard/Networktables
    // std::vector<double> data = frc::SmartDashboard::GetNumberArray("data", defaultVal);
    
    // No april tag data
    if (data.at(0) == -1){
        // foundTag_ = false;
        return;
    }
    // Format of
    //[         0            1    2   3   4     5        6       7 ]
    //[ apriltag or nah , tagid , x , y , _ , delay , dataid , zang]

    Point tagPos{data.at(2), data.at(3)}; //Robot position relative to tag

    double tagZAng = -data.at(4);
    // frc::SmartDashboard::PutNumber("Tag Ang", tagZAng * 180 / M_PI);
    int tagID = data.at(1);
    int uniqueVal = data.at(6);
    double delay = data.at(5) / 1000.0; //Seconds

    double navxTagZAng/*, tagAngToRobotAng*/;
    if (tagID <= 4 && tagID > 0){
        navxTagZAng = yaw_ + 90;
        // tagAngToRobotAng = (tagZAng * 180 / M_PI) - 90;
    }
    else if (tagID >= 5 && tagID < 9){
        navxTagZAng = yaw_ - 90;
        // tagAngToRobotAng = (tagZAng * 180 / M_PI) + 90;
    }
    else
    {
        return;
    }
    navxTagZAng = GeometryHelper::getPrincipalAng2Deg(navxTagZAng);

    navxTagZAng *= -(M_PI / 180.0);

    // frc::SmartDashboard::PutNumber("Tag x", tagX);
    // frc::SmartDashboard::PutNumber("Tag y", tagY);
    // frc::SmartDashboard::PutNumber("Tag ZAng", tagZAng);

    //check new tag
    bool sameTag = (uniqueVal == prevUniqueVal_);
    frc::SmartDashboard::PutBoolean("Different Tag", !sameTag);
    if(sameTag){
        if (robotX_ > 3.919857 + 2 && robotX_ < 12.621893 - 2) // If robot is not near community nor loading station
        {
            // foundTag_ = false;
        }
        return;
    }
    prevUniqueVal_ = uniqueVal; 

    //Tag id wonky
    if (tagID < 1 || tagID > 8) // Ignore for now
    {
        return;
    }

    //If robot tilted (on charge station)
    if (abs(tilt) > 5){
        return;
    }

    // if (tagID != prevTag_) // If tag changes
    // {
    //     prevTag_ = tagID; // Set to change, do nothing (skip a frame)
    //     return;
    // }
    // prevTag_ = tagID; // Set the past known tag to be this tag

    // Rotate the tag by the seen angle to orient displacement (since the cameras rotate with the robot <- undo that)
    // Now is oriented to a robot facing the tag directly
    Point orientedTagPos;
    if (!frc::DriverStation::IsDisabled()){
        //Using navx to orient tag displacement
        orientedTagPos = tagPos.rotateClockwise(navxTagZAng);
    }
    else{
        //Using tag data to orient tag displacement
        orientedTagPos = tagPos.rotateClockwise(tagZAng);
    }

    // Get xy of field tag position
    Point fieldTagPos = FieldConstants::TAG_XY[tagID - 1];

    // frc::SmartDashboard::PutNumber("GX", fieldTagX);
    // frc::SmartDashboard::PutNumber("GY", fieldTagY);

    // frc::SmartDashboard::PutNumber("TOX", orientedTagX);
    // frc::SmartDashboard::PutNumber("TOY", orientedTagY);

    //World coordinate from tag reading
    Point robotTagPos;
    // Adding the displacement from tag to robot + apriltag position
    if (tagID <= 4) {// Left side of field
        robotTagPos = fieldTagPos + orientedTagPos.rotateCounterclockwise90();
    }
    else{ // Right side of field
        robotTagPos = fieldTagPos + orientedTagPos.rotateClockwise90();
    }

    double xPos = robotTagPos.getX();
    double yPos = robotTagPos.getY();
    //April tag reading
    frc::SmartDashboard::PutNumber("AT X", xPos);
    frc::SmartDashboard::PutNumber("AT Y", yPos);

    if (xPos < 0 || yPos < 0 || xPos > FieldConstants::FIELD_LENGTH || yPos > FieldConstants::FIELD_WIDTH) // If rbot is out of bounds
    {
        return;
    }

    //Reject tag on the opposite side of the field
    //TagID 1-4 are on red side
    //TagID 5-8 are on blue side
    if(FieldConstants::onPlayerStationHalf(tagID >= 5, xPos)){
        return;
    }

    // 1st check = set it regardless
    if (!foundTag_)
    {
        robotX_ = xPos;
        robotY_ = yPos;
        foundTag_ = true;
    }
    else
    {
        // Update by moving the position to the calculated position if the position is close
        // Increment it proportionally by the error
        //  if (abs(robotX_ - xPos) < 1 && abs(robotY_ - yPos) < 1) // TODO watch out here
        //  {
        //      robotX_ += (-robotX_ + xPos) * 0.01;
        //      robotY_ += (-robotY_ + yPos) * 0.01;
        //  }

        // Point xyVel = getXYVel();
        // double vel = sqrt(xyVel.first * xyVel.first + xyVel.second * xyVel.second);

        // robotX_ += (-robotX_ + xPos) * (0.1 / (1 + 1 * vel));
        // robotY_ += (-robotY_ + yPos) * (0.1 / (1 + 1 * vel));

        //Find most recent pose
        double time = timer_.GetFPGATimestamp().value();
        auto historicalPose = prevPoses_.lower_bound(time - SwerveConstants::CAMERA_DELAY - delay);
        if (historicalPose != prevPoses_.end() && abs(historicalPose->first - (time - SwerveConstants::CAMERA_DELAY) < 0.007))
        {
            // frc::SmartDashboard::PutNumber("HX", historicalPose->second.first.first);
            // frc::SmartDashboard::PutNumber("HY", historicalPose->second.first.second);
            double vel = historicalPose->second.second.getMagnitude();
            
            double xDiff = xPos - historicalPose->second.first.getX();
            double yDiff = yPos - historicalPose->second.first.getY();
            // frc::SmartDashboard::PutNumber("DiffX", xDiff);
            // frc::SmartDashboard::PutNumber("DiffY", yDiff);

            double dist;
            if (xPos > FieldConstants::FIELD_LENGTH / 2)
            {
                dist = FieldConstants::FIELD_LENGTH - xPos;
            }
            else
            {
                dist = xPos;
            }
            double multiplier;
            if (dist > FieldConstants::FIELD_LENGTH / 2)
            {
                multiplier = 0;
            }
            else
            {
                multiplier = 0.75 * (1 - (dist / (FieldConstants::FIELD_LENGTH / 2)));
            }

            if (sqrt(xDiff * xDiff + yDiff * yDiff) > 2 && numLargeDiffs_ < 5 /* && !frc::DriverStation::IsDisabled()*/)
            {
                multiplier = 0;
                numLargeDiffs_ += 1;
            }
            else
            {
                numLargeDiffs_ = 0;
            }

            std::map<double, std::pair<Point, Vector>>::iterator it;
            for (it = prevPoses_.begin(); it != prevPoses_.end(); it++)
            {
                double mult = multiplier / (1.0 + 0.1 * vel);
                it->second.first.move({xDiff * mult, yDiff * mult});
            }

            robotX_ = prevPoses_.rbegin()->second.first.getX();
            robotY_ = prevPoses_.rbegin()->second.first.getY();

            // if (frc::DriverStation::IsDisabled())
            // {
            //     if (true)
            //     {
            //         yawTagOffset_ = (tagAngToRobotAng - yaw_);
            //     }
            // }
            // else
            // {
            //     if (getXYVel().first < 0.1 && getXYVel().second < 0.1 && abs(tagAngToRobotAng - yaw_) < 5 && abs(abs(yaw_) - 90) < 10 /* && IN FRONT OF CUBE OR SOMETHING*/)
            //     {
            //         yawTagOffset_ += 0.3 * (tagAngToRobotAng - yaw_);
            //     }
            // }
        }
    }
}

/// @brief gets the target scoring position
/// @param scoringLevel target scoring level [1, 9]
/// @return {wantedtX, wantedtY} or {0,0} if invalid data
Point SwerveDrive::checkScoringPos(int scoringLevel) // TODO get better values
{
    // If tag not found
    if (!foundTag_)
    {
        return {0, 0};
    }

    // if (robotX_ > 3.919857 + 2 && robotX_ < 12.621893 - 2) // If robot is not near community nor loading station
    // {
    //     return {0, 0};
    // }

    // If robot is out of bounds, odometry says out of field lmao
    if (robotX_ < 0 || robotY_ < 0 || robotX_ > FieldConstants::FIELD_LENGTH || robotY_ > FieldConstants::FIELD_WIDTH)
    {
        return {0, 0};
    }

    //Target scoring position
    double wantedX, wantedY;
    if(FieldConstants::onPlayerStationHalf(config_.isBlue, robotX_)){//Robot going to playerstation
        wantedX = FieldConstants::getPos(FieldConstants::PLAYER_STATION_X, config_.isBlue);

        wantedY = FieldConstants::TAG_XY[4].getY(); //Tag position
        // Check left or right player station, offset wantedY, plus a bit depending 
        if (robotY_ > FieldConstants::TAG_XY[4].getY()){
            wantedY += 0.838 + (config_.isBlue?-0.127:0.0);
        }
        else{
            wantedY += -0.838 + (config_.isBlue?0.0:0.127);
        }
    }
    else{//Going to score
        wantedX = FieldConstants::getPos(FieldConstants::SCORING_X, config_.isBlue);
        if (scoringLevel == 1){
            wantedX += config_.isBlue?0.1:-0.1; // scoot towards driverstation by 10 cm, 0.5
        }

        double yIndex = config_.isBlue? (9 - setTagPos_) : (setTagPos_ - 1);//Tag ordering is reversed on opposite side
        wantedY = yIndex * 0.5588 + 0.512826;

        if (setTagPos_ == 9){
            wantedY += (config_.isBlue?1.0:-1.0) * (0.0254 * 4); //Add 4 inches Y
            wantedX += (config_.isBlue?-1.0:1.0) * (0.0254 * 2); //Move away from driverstation 2 inches
        }
    }
    return Point{wantedX , wantedY} + LineupTrim_;

    // Below is which ever is in front of the robot, above is choosing

    // if (!foundTag_)
    // {
    //     return -1;
    // }

    // if (robotX_ > 3.919857 && robotX_ < 12.621893)
    // {
    //     return -1;
    // }

    // if (robotX_ < 0 || robotY_ < 0 || robotX_ > FieldConstants::FIELD_LENGTH || robotY_ > FieldConstants::FIELD_WIDTH)
    // {
    //     return -1;
    // }

    // if (config_.isBlue)
    // {
    //     if (robotX_ >= FieldConstants::TAG_XY[0][0]) //TODO make parameters better for all TAG_XY, probably make it check y first
    //     {
    //         if (robotY_ > FieldConstants::TAG_XY[2][1])
    //         {
    //             return 10;
    //         }
    //         else
    //         {
    //             return -1;
    //         }
    //     }
    //     else
    //     {
    //         if (robotY_ > FieldConstants::TAG_XY[2][1])
    //         {
    //             return -1;
    //         }
    //         else if(robotX_ > 3.919857)
    //         {
    //             return -1;
    //         }
    //         else
    //         {
    //             //int scoringPos = (int)round(robotY_ / 0.5588);
    //             int scoringPos = (int)round((robotY_ - (0.5*FieldConstants::TAG_XY[0][1])) / 0.5588);
    //             if (scoringPos == 10)
    //             {
    //                 scoringPos = 9; // TODO fix, yeah yeah
    //             }
    //             return scoringPos;
    //         }
    //     }
    // }
    // else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed)
    // {
    //     if (robotX_ <= FieldConstants::TAG_XY[5][0]) //TODO make parameters beter for all TAG_XY, probably make it check y first
    //     {
    //         if (robotY_ > FieldConstants::TAG_XY[2][1])
    //         {
    //             return 10;
    //         }
    //         else
    //         {
    //             return -1;
    //         }
    //     }
    //     else
    //     {
    //         if (robotY_ > FieldConstants::TAG_XY[2][1])
    //         {
    //             return -1;
    //         }
    //         else if(robotX_ < 12.621893)
    //         {
    //             return -1;
    //         }
    //         else
    //         {
    //             int scoringPos = (int)round(robotY_ / 0.5588);
    //             if (scoringPos == 10)
    //             {
    //                 scoringPos = 9; // TODO fix, yeah yeah
    //             }
    //             return scoringPos;
    //         }
    //     }
    // }

    // return -1;
}

void SwerveDrive::setScoringPos(int scoringPos)
{
    if (scoringPos != -1){
        setTagPos_ = scoringPos;
    }
    // frc::SmartDashboard::PutNumber("s", setTagPos_);
}

int SwerveDrive::getScoringPos(){
    return setTagPos_;
}

// void SwerveDrive::resetYawTagOffset()
// {
//     yawTagOffset_ = 0;
// }

// double SwerveDrive::getYawTagOffset()
// {
//     return 0;
//     return yawTagOffset_;
// }