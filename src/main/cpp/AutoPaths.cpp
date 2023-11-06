// If you're reading this file, I know it's a mess and terribly structured.
// I swear it was better before, but then we needed more curves and more paths
// and I had to add things in the middle of competitions and had no structure
// to work with and it's rushed etc etc.

#include "AutoPaths.h"

#include "Helpers/GeometryHelper.h"

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
    left_ = false;
    cubeIntaking_ = false;
    placingTimerStarted_ = false;
    comingDownChargingStation_ = false;
    taxied_ = false;
    dumbAutoDocking_ = false;
    onCharge_ = false;
    sendingIt_ = false;
    hitChargeStation_ = false;
    firstCubeArmSafety_ = false;
    isBlue_ = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;

    // frc::SmartDashboard::PutNumber("Min Vel" , 0.0);
    // frc::SmartDashboard::PutNumber("K Vel" , 1.0);

    // frc::SmartDashboard::PutNumber("X offset", 0.0);
    // frc::SmartDashboard::PutNumber("Y offset", 0.0);
}

void AutoPaths::setPath(Path path)
{
    path_ = path;
    pointNum_ = 0;
    swervePoints_.clear();
    nextPointReady_ = false;
    dumbTimerStarted_ = false;
    failsafeStarted_ = false;

    armStart_ = 0.0; //Time arm to place cube early
    armPlacing_ = false;

    taxiStart = 0.0; //Drive out a bit taxi
    taxiDriving_ = false;
    onCharge_ = false;

    //Set swerve points
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
        x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        yaw = FieldConstants::getForward(isBlue_);
        if (isBlue_)
        {
            if (left_){
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (!left_){
                y = FieldConstants::TOP_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y = FieldConstants::BOTTOM_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CUBE_MID:
    {
        double x, y, yaw;
        if (isBlue_)
        {
            x = FieldConstants::SCORING_X.blue;
            yaw = 90;
            if (left_)
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
            x = FieldConstants::SCORING_X.red;
            yaw = -90;
            if (!left_)
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_HIGH:
    {
        double x, y, yaw;
        yaw = (isBlue_?1.0:-1.0) * (90.0);

        x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        bool top = !(isBlue_ ^ left_);
        if(top){
            yaw += (isBlue_? 0.0 : 5.0); //Rotate since robot collides with wall, only red
            x += (isBlue_? -2.0 : 1.5) * 0.0254; //Move towards driver red, away blue
            y = FieldConstants::TOP_CONE_Y;
            y += (isBlue_? 0.0 : -4.0) * 0.0254; //Shift out a bit away from wall, only for red since it rotates
        }
        else{
            yaw += (isBlue_? 5.0 : 0.0); //Rotate since robot collides with wall, only blue
            x += (isBlue_? 1.0 : 2.5) * 0.0254; //Move towards from driver red, away blue
            y = FieldConstants::BOTTOM_CONE_Y;
            y += (isBlue_? 5.0 : 0.0) * 0.0254; //Shift out a bit away from wall, blue avoid wall
        }
        y += ((isBlue_? 1.0 : -1.0) * -SwerveConstants::CLAW_MID_OFFSET); //Account for arm offset

        //x += frc::SmartDashboard::GetNumber("X offset", 0.0) * 0.0254;
        //y += frc::SmartDashboard::GetNumber("Y offset", 0.0) * 0.0254;

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);
        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        double x, y, yaw;
        if (isBlue_)
        {
            x = FieldConstants::SCORING_X.blue;
            yaw = 90;
            if (left_)
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
            x = FieldConstants::SCORING_X.red;
            yaw = -90;
            if (!left_)
            {
                y = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        double x, y, yaw;
        yaw = (isBlue_?1.0:-1.0) * (90.0);

        x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        bool top = !(isBlue_ ^ left_);
        if(top){
            yaw += (isBlue_? 0.0 : 0.0); //Rotate since robot collides with wall, only red
            x += (isBlue_? 0.0 : 0.0) * 0.0254; //Move towards driver red, away blue
            y = FieldConstants::TOP_MIDDLE_CONE_Y;
            y += (isBlue_? 0.0 : 0.0) * 0.0254; //Shift out a bit away from wall, only for red since it rotates
        }
        else{
            yaw += (isBlue_? 5.0 : 0.0); //Rotate since robot collides with wall, only blue
            x += (isBlue_? 0.0 : 0.0) * 0.0254; //Move towards from driver red, away blue
            y = FieldConstants::BOTTOM_MIDDLE_CONE_Y;
            y += (isBlue_? 0.0 : 0.0) * 0.0254; //Shift out a bit away from wall, blue avoid wall
        }
        y += ((isBlue_? 1.0 : -1.0) * -SwerveConstants::CLAW_MID_OFFSET); //Account for arm offset

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        double x, y, yaw;
        if (isBlue_)
        {
            x = FieldConstants::SCORING_X.blue;
            yaw = 90;
            if (left_){
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            x = FieldConstants::SCORING_X.red;
            yaw = -90;
            if (!left_){
                y = FieldConstants::TOP_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        frc::SmartDashboard::PutNumber("Target Position X", x);
        frc::SmartDashboard::PutNumber("Target Position Y", y);

        swervePoints_.push_back(SwervePose(x, y, yaw, 0));
        break;
    }
    case FIRST_CUBE_HIGH:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        x1 = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        x2 = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        if (isBlue_){
            yaw1 = 90;
            yaw2 = 90;
            if (left_){
                y1 = FieldConstants::TOP_PIECE_Y - SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::TOP_CUBE_Y /* - SwerveConstants::CLAW_MID_OFFSET*/ + 0.33;
                yaw2 = 104;
            }
            else{
                y1 = FieldConstants::BOTTOM_PIECE_Y - SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::BOTTOM_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else{
            yaw1 = -90;
            yaw2 = -90;
            if (!left_){
                y1 = FieldConstants::TOP_PIECE_Y + SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::TOP_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y1 = FieldConstants::BOTTOM_PIECE_Y + SwerveConstants::CLAW_MID_OFFSET;
                y2 = FieldConstants::BOTTOM_CUBE_Y /* + SwerveConstants::CLAW_MID_OFFSET*/ - 0.33;
                yaw2 = -76;
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
        x1 = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        x2 = FieldConstants::getPos(FieldConstants::AUTO_DOCK_X, isBlue_);
        if (isBlue_){
            yaw1 = 90;
            yaw2 = 90;
            if (left_){
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else{
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }
        else{
            yaw1 = -90;
            yaw2 = -90;
            if (!left_){
                y1 = FieldConstants::TOP_PIECE_Y;
            }
            else{
                y1 = FieldConstants::BOTTOM_PIECE_Y;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0.1));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 1.5)); //~5.2 is dist
        break;
    }
    case SECOND_CUBE_MID:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        double forward = isBlue_? 1.0 : -1.0; //Forward direction
        yaw1 = forward * 90.0;
        yaw2 = forward * 90.0;
        x1 = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        x2 = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        bool top = !(isBlue_ ^ left_);
        if(top){
            y1 = FieldConstants::TOP_MID_PIECE_Y;
            y2 = FieldConstants::TOP_CUBE_Y;
        }
        else{
            y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
            y2 = FieldConstants::BOTTOM_CUBE_Y;
        }
        if(left_){ //Left
            yaw1 += -60.0; //Rotate towards cube
            yaw2 += 25; //Rotate more on right to place
        }
        else{
            yaw1 += 45.0; //Rotate towards cube
            y2 += forward * 0.444; //Shift placing pos
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE_HIGH:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        yaw2 = (isBlue_ ? 1.0:-1.0) * 90.0;
        x1 = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        x2 = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        bool top = !(isBlue_ ^ left_);
        if (isBlue_){
            if (left_){
                yaw1 = 65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                yaw1 = 125;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else{
            if (!left_){
                yaw1 = -65;
                y1 = FieldConstants::TOP_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                yaw1 = -115;
                y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
                y2 = FieldConstants::MID_CUBE_Y + SwerveConstants::CLAW_MID_OFFSET;
            }
        }

        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0));

        break;
    }
    case SECOND_CUBE_DOCK:
    {
        double x1, x2, y1, y2, yaw1, yaw2;
        double forward = isBlue_? 1.0 : -1.0; //Forward direction
        yaw1 = forward * 90.0;
        yaw2 = forward * 90.0;
        x1 = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        x2 = FieldConstants::getPos(FieldConstants::AUTO_DOCK_X, isBlue_);
        x2 += forward * (- 0.33 - 0.33 - 0.8); //Move towards charge more
        y2 = FieldConstants::AUTO_DOCK_Y;
        bool top = !(isBlue_ ^ left_);
        if(top){
            y1 = FieldConstants::TOP_MID_PIECE_Y;
        }
        else{
            y1 = FieldConstants::BOTTOM_MID_PIECE_Y;
        }
        if(left_){
            yaw1 += -60.0; //Rotate 60 degrees towards cube
            yaw2 += -89.9; //Have gearbox face charge station
        }
        else{
            yaw1 += 45.0; //Rotate 60 degrees towards cube
            yaw2 += 89.9; //Have gearbox face charge station (89.9 for turning logic)
        }
        swervePoints_.push_back(SwervePose(x1, y1, yaw1, 0));
        swervePoints_.push_back(SwervePose(x2, y2, yaw2, 0.5));
        break;
    }
    case SECOND_CUBE_GRAB:
    {
        double x, y, yaw;
        x = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
        double forward = isBlue_? 1.0 : -1.0; //Forward direction
        yaw = forward * 90.0;
        bool top = !(isBlue_ ^ left_);
        if(top){
            y = FieldConstants::TOP_MID_PIECE_Y;
        }
        else{
            y = FieldConstants::BOTTOM_MID_PIECE_Y;
        }
        if(left_){//Right
            yaw += -60.0; //Rotate towards cube
        }
        else{
            yaw += 45.0; //Rotate towards cube
        }
        swervePoints_.push_back(SwervePose(x, y, yaw, 0));

        break;
    }
    case AUTO_DOCK:
    {
        double x, y, yaw;
        y = FieldConstants::AUTO_DOCK_Y;
        x = FieldConstants::getPos(FieldConstants::AUTO_DOCK_X, isBlue_);
        // yaw = 0;
        if (isBlue_)
        {
            x += 1;
            yaw = 0; // was 90
        }
        else
        {
            x += -1;
            yaw = 179.99; // was -90
        }
        swervePoints_.push_back(SwervePose(x, y, yaw, 0.5));
        break;
    }
    case TAXI_DOCK_DUMB:{break;}
    case NO_TAXI_DOCK_DUMB:{break;}
    case NOTHING:{break;}
    case DRIVE_BACK_DUMB:{break;}
    case WAIT_5_SECONDS:{break;}
    }

    pathSet_ = true;
    pathGenerated_ = false;
    curveSecondStageGenerated_ = false;
    yawStageGenerated_ = false;
}

AutoPaths::Path AutoPaths::getPath(){
    return path_;
}

void AutoPaths::setActions(Path a1, Path a2, Path a3, Path a4, bool slow){
    actions_.clear();
    actions_.push_back(a1);
    actions_.push_back(a2);
    actions_.push_back(a3);
    actions_.push_back(a4);

    isBlue_ = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    //bool overCableBump = (isBlue_ ^ left_);

    //firstCubeArmSafety_ = overCableBump;
    //slowTraj_ = overCableBump;
    firstCubeArmSafety_ = slow;
    slowTraj_ = slow;
 
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
    placingTimerStarted_ = false;

    comingDownChargingStation_ = false;
    taxied_ = false;
    dumbAutoDocking_ = false;
    onCharge_ = false;
    sendingIt_ = false;
    hitChargeStation_ = false;

    forward_ = true;

    switch (a1)
    {
        case PRELOADED_CONE_HIGH:
        case PRELOADED_CUBE_HIGH:
        case PRELOADED_CONE_HIGH_MIDDLE:
            armPosition_ = TwoJointArmProfiles::HIGH;
            forward_ = true;
            break;
        case PRELOADED_CONE_MID:
        case PRELOADED_CONE_MID_MIDDLE:
            armPosition_ = TwoJointArmProfiles::MID;
            forward_ = true;
            break;
        case PRELOADED_CUBE_MID:
            armPosition_ = TwoJointArmProfiles::CUBE_MID;
            forward_ = true;
            break;
        case AUTO_DOCK:
        case TAXI_DOCK_DUMB:
        case NO_TAXI_DOCK_DUMB:
        case NOTHING:
        case DRIVE_BACK_DUMB:
        case WAIT_5_SECONDS:
        default:
            armPosition_ = TwoJointArmProfiles::STOWED;
            forward_ = true;
    }
    //frc::SmartDashboard::PutBoolean("SLOW TRAJ", slowTraj_);
}

void AutoPaths::startTimer(){
    startTime_ = timer_.GetFPGATimestamp().value();
}

void AutoPaths::startAutoTimer(){
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
    isBlue_ = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    //frc::SmartDashboard::PutBoolean("Auto Is Blue", isBlue_);
    if (!actionsSet_){
        return;
    }

    if (!pathSet_){
        if (actionNum_ > static_cast<int>(actions_.size()) - 1){
            swerveDrive_->drive({0, 0}, 0);
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

    //Generating Non dumb trajectories
    if (path_ != DRIVE_BACK_DUMB && path_ != NOTHING && path_ != WAIT_5_SECONDS && path_ != TAXI_DOCK_DUMB && path_ != NO_TAXI_DOCK_DUMB)
    {
        SwervePose pose;
        for (size_t i = pointNum_; i < swervePoints_.size(); ++i)
        {
            // pose = swervePoints_[i].getPose(time, pointOver);
            if (!pathGenerated_ ||
                ((  path_ == SECOND_CUBE_MID ||
                    path_ == FIRST_CUBE_HIGH || 
                    path_ == AUTO_DOCK || 
                    (path_ == FIRST_CUBE_DOCK && pointNum_ == 1) || 
                    (path_ == SECOND_CUBE_DOCK /* && pointNum_ == 0*/) || 
                    path_ == SECOND_CUBE_GRAB)
                    && 
                    (!curveSecondStageGenerated_ || !yawStageGenerated_)))
            {
                SwervePose currPose(swerveDrive_->getX(), swerveDrive_->getY(), swerveDrive_->getYaw(), 0);
                if (path_ == SECOND_CUBE_MID) // COULDO make the if logic not completely terrible
                {
                    if (!pathGenerated_){//Trajectory generation
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().getX());
                        generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().getX());
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        if (pointNum_ == 0){ //Going to 2nd cube
                            double setY;
                            if (isBlue_){
                                if (left_){
                                    // setY = 4.8;
                                    setY = FieldConstants::TOP_CONE_Y - 0.18 /* + 0.1*/;
                                }
                                else{
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.18 /* - 0.1*/;
                                }
                            }
                            else{
                                if (left_){
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.18 /* - 0.1*/;
                                }
                                else{
                                    setY = FieldConstants::TOP_CONE_Y - 0.18 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());

                            double setYaw;
                            if (isBlue_){
                                setYaw = 90;
                            }
                            else{
                                setYaw = -90;
                            }
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                            yawStageGenerated_ = true;
                        }
                        else{ //Coming back with 2nd cube
                            double setY = swervePoints_[i].y;
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                            curveSecondStageGenerated_ = true;

                            double setYaw = swervePoints_[i].yaw;
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                            yawStageGenerated_ = true;
                        }
                    }

                    Pose1D yProfile = getYProfile();
                    //Curve around charge stations, initialize y profile only after going past it
                    bool curveReady = false;
                    if (pointNum_ == 0){ //First point curve when going out
                        if (isBlue_){
                            curveReady = (swerveDrive_->getX() > 4.8514); // 2.921
                        }
                        else{
                            curveReady = (swerveDrive_->getX() < 11.688318); // 13.621893
                        }
                    }
                    else //Else curve when coming back
                    {
                        if (isBlue_){
                            curveReady = (swerveDrive_->getX() < 4.8514);
                        }
                        else{
                            curveReady = (swerveDrive_->getX() > 11.688318);
                        }
                    }

                    // Generate curved profile (y) after leaving charge station and after 1st profile is done
                    if ((!curveSecondStageGenerated_) &&
                        (/*pointNum_ == 1 || */ (pointNum_ == 0 &&
                                                curveReady && 
                                                timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.5 && 
                                                isStationary(yProfile))))
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0);
                        yawStageGenerated_ = true;
                    }
                    else if ((!curveSecondStageGenerated_) && ((pointNum_ == 1 && curveReady && isStationary(yProfile))))
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        curveSecondStageGenerated_ = true;
                    }

                    // if (curveSecondStageGenerated_ && !yawStageGenerated_ && pointNum_ == 1)
                    // {

                    //     if (get<0>(yProfile) == 0 && get<1>(yProfile) == 0)
                    //     {
                    //         double setYaw = swervePoints_[i].yaw;
                    //         yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                    //         yawStageGenerated_ = true;
                    //     }
                    // }
                }
                else if (path_ == FIRST_CUBE_HIGH)
                {
                    if (!pathGenerated_)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().getX());
                        generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().getX());
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0);
                        yawStageGenerated_ = true;

                        if (/*pointNum_ == 0*/ true)
                        {
                            double setY;
                            if (isBlue_){
                                if (left_){
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 - 0.0254 * 2 /* + 0.1*/;
                                }
                                else{
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                            }
                            else
                            {
                                if (left_)
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 - 0.0254 * 2 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        }
                        // else
                        // {
                        //     double setY = swervePoints_[i].y;
                        //     yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
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
                            if (isBlue_)
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
                            if (isBlue_)
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
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                            curveSecondStageGenerated_ = true;
                        }
                    }
                }
                else if (path_ == AUTO_DOCK)
                {
                    if (!pathGenerated_)
                    {
                        double setX;
                        if (isBlue_)
                        {
                            setX = FieldConstants::SCORING_X.blue + 0.3;
                        }
                        else
                        {
                            setX = FieldConstants::SCORING_X.red - 0.3;
                        }
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());

                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();
                        yawStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.2)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0);
                        yawStageGenerated_ = true;
                    }
                }
                else if ((path_ == FIRST_CUBE_DOCK) && pointNum_ == 1)
                {
                    if (!pathGenerated_)
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 

                        curveSecondStageGenerated_ = false;
                        pathGenerated_ = true;
                    }

                    if ((!curveSecondStageGenerated_) && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1)
                    {
                        double setX = swervePoints_[i].x;
                        // xTraj_.generateTrajectory(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());
                        generateXTraj(swerveDrive_->getX(), setX, swerveDrive_->getXYVel().getX());
                        curveSecondStageGenerated_ = true;
                    }
                }
                else if ((path_ == SECOND_CUBE_DOCK || path_ == SECOND_CUBE_GRAB) /* && pointNum_ == 0*/){
                    if (pointNum_ == 0)
                    {
                        if (!pathGenerated_)
                        {
                            double setX = swervePoints_[i].x;
                            // xTraj_.generateTrajectory(currPose.x, setX, swerveDrive_->getXYVel().getX());
                            generateXTraj(currPose.x, setX, swerveDrive_->getXYVel().getX());
                            curveSecondStageStartTime_ = timer_.GetFPGATimestamp().value();

                            // double setYaw = swervePoints_[i].yaw;
                            // yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                            double setYaw;
                            if (isBlue_){
                                setYaw = 90;
                            }
                            else{
                                setYaw = -90;
                            }
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                            yawStageGenerated_ = true;

                            // double setY = swervePoints_[i].y;
                            double setY;
                            if (isBlue_){
                                if (left_)
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 /* + 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                            }
                            else{
                                if (left_)
                                {
                                    setY = FieldConstants::BOTTOM_CONE_Y + 0.05 /* - 0.1*/;
                                }
                                else
                                {
                                    setY = FieldConstants::TOP_CONE_Y - 0.05 /* + 0.1*/;
                                }
                            }
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());

                            curveSecondStageGenerated_ = false;
                            pathGenerated_ = true;
                        }

                        // frc::SmartDashboard::PutBoolean("GEN", curveSecondStageGenerated_);
                        // frc::SmartDashboard::PutBoolean("PATH", pathGenerated_);
                        // frc::SmartDashboard::PutNumber("TIME", timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_);
                        // Pose1D yProfile = yTraj_.getProfile();
                        Pose1D yProfile = getYProfile();

                        bool curveReady = false;
                        if (isBlue_){
                            curveReady = (swerveDrive_->getX() > 4.8514); // 2.921
                        }
                        else{
                            curveReady = (swerveDrive_->getX() < 11.688318); // 13.621893
                        }
                        if ((!curveSecondStageGenerated_) /* && timer_.GetFPGATimestamp().value() - curveSecondStageStartTime_ > 1.5*/ && curveReady && isStationary(yProfile))
                        {
                            double setY = swervePoints_[i].y;
                            // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                            generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                            curveSecondStageGenerated_ = true;

                            double setYaw = swervePoints_[i].yaw;
                            yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
                        }
                    }
                    else
                    {
                        double setY = swervePoints_[i].y;
                        // yTraj_.generateTrajectory(swerveDrive_->getY(), setY, swerveDrive_->getXYVel().getY());
                        generateYTraj(currPose.y, setY, swerveDrive_->getXYVel().getY());
                        curveSecondStageGenerated_ = true;

                        double setYaw = swervePoints_[i].yaw;
                        yawTraj_.generateTrajectory(currPose.yaw, setYaw, 0); // 
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

            if (path_ == SECOND_CUBE_MID)
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
                //     //  frc::SmartDashboard::PutNumber("YVel", swerveDrive_->getXYVel().getY());
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
            else if (path_ == FIRST_CUBE_HIGH){
                Pose1D xProfile = getXProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();

                pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                if (isStationary(pose)){
                    pointOver = true;
                }
            }
            else if (path_ == AUTO_DOCK)
            {
                // double heldX = (isBlue_) ? FieldConstants::SCORING_X.blue + 0.05 : FieldConstants::SCORING_X.red - 0.05;
                Pose1D xProfile = getXProfile();
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
                    if (isBlue_)
                    {
                        xVel = SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    else
                    {
                        xVel = -SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    pose = SwerveFromPose1D({swerveDrive_->getX(), xVel, 0}, yProfile, yawProfile);

                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (isBlue_)
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
            else if ((path_ == SECOND_CUBE_DOCK || path_ == SECOND_CUBE_GRAB) /* && pointNum_ == 0*/){
                Pose1D xProfile = getXProfile();
                Pose1D yProfile = getYProfile();
                Pose1D yawProfile = yawTraj_.getProfile();
                // if (curveSecondStageGenerated_)
                // {
                //     yProfile = yTraj_.getProfile();
                // }
                if (pointNum_ == 0){
                    pose = SwerveFromPose1D(xProfile, yProfile, yawProfile);
                    if (isStationary(pose)){
                        pointOver = true;
                    }
                }
                else if (!hitChargeStation_){
                    double xVel;
                    if (isBlue_){
                        xVel = -SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    else{
                        xVel = SwerveConstants::PRE_SENDING_IT_SPEED * SwerveConstants::MAX_TELE_VEL;
                    }
                    // frc::SmartDashboard::PutNumber("Y DOCK VEL", get<1>(yProfile));
                    // frc::SmartDashboard::PutNumber("YAW DOCK VEL", get<1>(yawProfile));
                    pose = SwerveFromPose1D({swerveDrive_->getX(), xVel, 0}, yProfile, yawProfile);
                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (isBlue_){
                        if (tilt > 5){
                            hitChargeStation_ = true;
                        }
                    }
                    else{
                        if (tilt < -5){
                            hitChargeStation_ = true;
                        }
                    }

                    frc::SmartDashboard::PutBoolean("Hit Charge Station", false);
                }
                else
                {
                    frc::SmartDashboard::PutBoolean("Hit Charge Station", true);
                }
            }
            else if ((path_ == FIRST_CUBE_DOCK && pointNum_ == 1))
            {
                double heldX = FieldConstants::getPos(FieldConstants::PIECE_X, isBlue_);
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
                if (isStationary(pose)){
                    pointOver = true;
                }
            }
            else{
                pose = currPath_.getPose(time, pointOver);
            }

            if (!pointOver){
                break;
            }
            else{
                if (nextPointReady_ && i == swervePoints_.size() - 1){
                    pathSet_ = false;
                    nextPointReady_ = false;
                    ++actionNum_;
                    // pathGenerated_ = false;
                    // curveSecondStageGenerated_ = false;
                    // yawStageGenerated_ = false;
                    startTimer();
                    return;
                }
                else if (i == swervePoints_.size() - 1){
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
            //If ended and docked
            if (timer_.GetFPGATimestamp().value() - autoStartTime_ > 14.9 &&
                (path_ == SECOND_CUBE_DOCK || path_ == FIRST_CUBE_DOCK || path_ == AUTO_DOCK))
            {
                swerveDrive_->lockWheels();
            }
            else if (!pointOver || pointNum_ != 1 || path_ != SECOND_CUBE_DOCK){
                // frc::SmartDashboard::PutNumber("VEL X EXPECTED", pose.xVel);
                // frc::SmartDashboard::PutNumber("VEL Y EXPECTED", pose.yVel);
                if(abs(pose.xVel) < 0.25){ //Compensate for overshoot
                    pose.xVel *= -0.75;
                }
                if(abs(pose.yVel) < 0.25){ //Compensate for undershoot
                    pose.yVel *= 2.0;
                }
                swerveDrive_->drivePose(pose);
            }
        }
    }
    //Dumb paths just have a timer
    else if (path_ == WAIT_5_SECONDS){
        if (nextPointReady_){
            pathSet_ = false;
            nextPointReady_ = false;
            ++actionNum_;
            startTimer();
            return;
        }
    }
    else{
        if (!dumbTimerStarted_){
            timer_.Stop();
            timer_.Reset();
            timer_.Start();
            dumbTimerStarted_ = true;
        }
    }

    //Arm position and drive logic, also ends path after placed
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
        armPosition_ = TwoJointArmProfiles::HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::HIGH && arm_->getState() == TwoJointArm::HOLDING_POS){
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_){
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            //Place for 0.3 seconds
            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.3){
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else{
            clawOpen_ = false;
        }

        break;
    }
    case PRELOADED_CUBE_HIGH:
    {
        forward_ = true;
        clawOpen_ = true;
        armPosition_ = TwoJointArmProfiles::CUBE_HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::CUBE_HIGH && arm_->getState() == TwoJointArm::HOLDING_POS){
            wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            if (!placingTimerStarted_){
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.4){
                pointOver = true;
                nextPointReady_ = true;
            }
        }
        else{
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
        }

        break;
    }
    case PRELOADED_CONE_HIGH_MIDDLE:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::HIGH;

        if (arm_->getPosition() == TwoJointArmProfiles::HIGH && arm_->getState() == TwoJointArm::HOLDING_POS){
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_){
                placingStartTime_ = timer_.GetFPGATimestamp().value();
                placingTimerStarted_ = true;
            }

            if (placingTimerStarted_ && (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.7)){
                clawOpen_ = true;
                placingTimerStarted_ = false;
                nextPointReady_ = true;
            }
        }
        else{
            clawOpen_ = false;
        }

        break;
    }
    case PRELOADED_CONE_MID_MIDDLE:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        cubeIntaking_ = false;
        armPosition_ = TwoJointArmProfiles::MID;

        if (arm_->getPosition() == TwoJointArmProfiles::MID && arm_->getState() == TwoJointArm::HOLDING_POS)
        {
            // wheelSpeed_ = ClawConstants::OUTAKING_SPEED;
            clawOpen_ = true;
            if (!placingTimerStarted_){
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
    case FIRST_CUBE_HIGH:
    {
        if (pointNum_ == 0){
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            if (pointOver){
                nextPointReady_ = true;
            }
        }
        else{ //Coming back to score high
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            bool armReady;
            if (isBlue_){
                armReady = (swerveDrive_->getX() < 2.921 - 0.2);
            }
            else{
                armReady = (swerveDrive_->getX() > 13.621893 + 0.2);
            }
            clawOpen_ = true;
            if (firstCubeArmSafety_){
                if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && !armReady){
                    cubeIntaking_ = false;
                    forward_ = false;
                    armPosition_ = TwoJointArmProfiles::STOWED;
                }
                else if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2 && armReady){
                    cubeIntaking_ = false;
                    forward_ = true;
                    armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
                }
                else{
                    forward_ = false;
                    clawOpen_ = true;
                    cubeIntaking_ = true;
                }
            }
            else{ //Fast Path
                if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2){ //Start moving arm before reaching placing location
                    cubeIntaking_ = false;
                    forward_ = true;
                    armPosition_ = TwoJointArmProfiles::CUBE_HIGH;
                }
                else{
                    forward_ = false;
                    clawOpen_ = true;
                    cubeIntaking_ = true;
                }
            }
            if(!armPlacing_ && armReady){
                armStart_ = timer_.GetFPGATimestamp().value();
                armPlacing_ = true;
            }
            if(armPlacing_ && (timer_.GetFPGATimestamp().value() - armStart_ > 1.0)){ // Score some time after arm extends
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED * 2.0; // Increase speed
            }
            if (arm_->getPosition() == TwoJointArmProfiles::CUBE_HIGH && arm_->getState() == TwoJointArm::HOLDING_POS && pointOver){
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED * 2.0; // Increase speed outtake
                if (!placingTimerStarted_){
                    placingStartTime_ = timer_.GetFPGATimestamp().value();
                    placingTimerStarted_ = true;
                }

                if (timer_.GetFPGATimestamp().value() - placingStartTime_ > 0.2) // Place for 0.2 seconds
                {
                    nextPointReady_ = true;
                    placingTimerStarted_ = false;
                }
            }
        }

        break;
    }
    case FIRST_CUBE_DOCK:
    {
        if (pointNum_ == 0){
            armPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            forward_ = false;
            cubeIntaking_ = true;
            clawOpen_ = true;
            if (pointOver){
                nextPointReady_ = true;
            }
        }
        else{
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.2){
                cubeIntaking_ = false;
                forward_ = false;
            }

            if (pointOver && timer_.GetFPGATimestamp().value() - autoStartTime_ < 14.9){
                double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE){
                    swerveDrive_->lockWheels();
                }
                else{
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive({output, 0}, 0);
                }
            }
        }

        break;
    }
    case SECOND_CUBE_MID:
    {
        if (pointNum_ == 0){
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
            if (isBlue_)
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
                wheelSpeed_ = ClawConstants::OUTAKING_SPEED * 2.0;
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
    case SECOND_CUBE_HIGH:
    {
        break;
    }
    case SECOND_CUBE_DOCK:
    {
        if (pointNum_ == 0){ //Grab second cube
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
        else{ //Dock
            wheelSpeed_ = ClawConstants::INTAKING_SPEED;
            clawOpen_ = true;
            if (timer_.GetFPGATimestamp().value() - startTime_ > 0.4){
                cubeIntaking_ = false;
                forward_ = false;
                armPosition_ = TwoJointArmProfiles::STOWED;
            }

            if (hitChargeStation_ && timer_.GetFPGATimestamp().value() - autoStartTime_ < 14.9){
                if (!sendingIt_){
                    sendingItTime_ = timer_.GetFPGATimestamp().value();
                    sendingIt_ = true;
                }
                double time = timer_.GetFPGATimestamp().value() - sendingItTime_;
                if (time < SwerveConstants::SENDING_IT_TIME){ //Go fast for a bit
                    double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                    double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (isBlue_){
                        if (abs(tilt) < SwerveConstants::MIN_TILT_ON_STATION){
                            swerveDrive_->drive({-SwerveConstants::SENDING_IT_FAST_SPEED, 0}, 0);
                        }
                        else{
                            swerveDrive_->drive({-SwerveConstants::SENDING_IT_MED_SPEED, 0}, 0);
                        }
                    }
                    else
                    {
                        if (abs(tilt) < SwerveConstants::MIN_TILT_ON_STATION){
                            swerveDrive_->drive({SwerveConstants::SENDING_IT_FAST_SPEED, 0}, 0);
                        }
                        else{
                            swerveDrive_->drive({SwerveConstants::SENDING_IT_MED_SPEED, 0}, 0);
                        }
                    }
                }
                else{ //Auto balance
                    double ang = (yaw_)*M_PI / 180.0;                                                          // Radians
                    double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                    double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                    double tilt = pitch * sin(ang) - roll * cos(ang);
                    if (abs(tilt) < SwerveConstants::AUTODEADANGLE){ 
                        frc::SmartDashboard::PutBoolean("Balanced", true);
                        swerveDrive_->lockWheels();
                    }
                    else{
                        frc::SmartDashboard::PutBoolean("Balanced", false);
                        double output = -SwerveConstants::AUTOKTILT * tilt;
                        swerveDrive_->drive({output, 0}, 0);
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
                double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (isBlue_)
                {
                    if (abs(tilt) < 12)
                    {
                        swerveDrive_->drive({SwerveConstants::SENDING_IT_FAST_SPEED, 0}, 0);
                    }
                    else
                    {
                        swerveDrive_->drive({SwerveConstants::SENDING_IT_MED_SPEED, 0}, 0);
                    }
                }
                else
                {
                    if (abs(tilt) < 12)
                    {
                        swerveDrive_->drive({-SwerveConstants::SENDING_IT_FAST_SPEED, 0}, 0);
                    }
                    else
                    {
                        swerveDrive_->drive({-SwerveConstants::SENDING_IT_MED_SPEED, 0}, 0);
                    }
                }
            }
            else
            {
                double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
                double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
                double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
                double tilt = pitch * sin(ang) - roll * cos(ang);
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE)
                {
                    swerveDrive_->lockWheels();
                }
                else
                {
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive({output, 0}, 0);
                }
            }
        }
        break;
    }
    case TAXI_DOCK_DUMB:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        armPosition_ = TwoJointArmProfiles::STOWED;

        //Let time for arm to come down
        if(timer_.Get().value() > 1.5){
            double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
            double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
            double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
            double tilt = pitch * sin(ang) - roll * cos(ang);

            double forward = isBlue_? 1.0:-1.0;
            if (isBlue_){
                if (tilt > 5){
                    comingDownChargingStation_ = true;
                }
            }
            else{
                if (tilt < -5){
                    comingDownChargingStation_ = true;
                }
            }

            if (!comingDownChargingStation_){ //Drive over charge to come down
                double vel = forward * SwerveConstants::SENDING_IT_FAST_SPEED;
                swerveDrive_->drive({vel, 0}, 0);
            }
            else if (comingDownChargingStation_ && !taxied_){
                double vel = forward * 0.2; //Slowly roll out
                swerveDrive_->drive({vel, 0}, 0);
                taxied_ = (abs(tilt) < 1);
            }
            else{
                if(!taxiDriving_){ //Set countdown for taxi wait
                    taxiStart = timer_.GetFPGATimestamp().value();
                    taxiDriving_ = true;
                }
                if(timer_.GetFPGATimestamp().value() - taxiStart < 1.5){ //Drive out for a bit
                    swerveDrive_->drive({forward * 0.1, 0}, 0);
                }
                else if(timer_.GetFPGATimestamp().value() - taxiStart < 3.5){ //Wait
                    swerveDrive_->drive({0.0, 0.0}, 0.0);
                }
                else{
                    if (!dumbAutoDocking_){ // Drive back into charge station
                        if (abs(tilt) > 10){
                            dumbAutoDocking_ = true;
                        }
                        else{
                            double vel = forward * -SwerveConstants::SENDING_IT_FAST_SPEED;
                            swerveDrive_->drive({vel, 0}, 0);
                        }
                    }

                    if (dumbAutoDocking_){ // Auto dock
                        if (abs(tilt) < SwerveConstants::AUTODEADANGLE){
                            swerveDrive_->lockWheels();
                        }
                        else{
                            double output = -SwerveConstants::AUTOKTILT * tilt;
                            swerveDrive_->drive({output, 0}, 0);
                        }
                    }
                }
            }
        }
        break;
    }
    case NO_TAXI_DOCK_DUMB:
    {
        forward_ = true;
        wheelSpeed_ = 0;
        armPosition_ = TwoJointArmProfiles::STOWED;

        //Let time for arm to come down
        double time = timer_.Get().value();
        if(time > 1.5){
            double ang = (yaw_)*M_PI / 180.0;                                                   // Radians
            double pitch = GeometryHelper::getPrincipalAng2Deg(pitch_ + SwerveConstants::PITCHOFFSET); // Degrees
            double roll = GeometryHelper::getPrincipalAng2Deg(roll_ + SwerveConstants::ROLLOFFSET);    // Degrees
            double tilt = pitch * sin(ang) - roll * cos(ang);

            double forward = isBlue_? 1.0:-1.0;

            time -= 1.5; //Start at 0
            if(abs(tilt) > 10){
                onCharge_ = true;
            }

            if (!onCharge_){ //Drive over charge to come down
                double vel = forward * SwerveConstants::SENDING_IT_FAST_SPEED;
                double forwardVel = (sin(sin(time - 3.0) + 2.3) + cos(time - 0.9))/2.0; //Goofy desmos graph
                vel *= forwardVel;
                swerveDrive_->drive({vel, 0}, 0);
            }
            else{
                if (abs(tilt) < SwerveConstants::AUTODEADANGLE){ // Auto dock
                    swerveDrive_->lockWheels();
                }
                else{
                    double output = -SwerveConstants::AUTOKTILT * tilt;
                    swerveDrive_->drive({output, 0}, 0);
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

        if (timer_.Get().value() < 2){
            double vel = (isBlue_? 1.0 : -1.0) * 0.2; //Drive away from driverstation
            swerveDrive_->drive({vel, 0}, 0);
        }
        else{
            swerveDrive_->drive({0, 0}, 0);
        }

        break;
    }
    case WAIT_5_SECONDS:
    {
        swerveDrive_->lockWheels();
        if (!failsafeStarted_){
            failsafeStarted_ = true;
            failsafeTimer_.Reset();
            failsafeTimer_.Start();
        }

        if (failsafeTimer_.Get().value() > 5){
            failsafeStarted_ = false;
            nextPointReady_ = true;
        }
        break;
    }
    }
}

void AutoPaths::setGyros(double yaw, double pitch, double roll)
{
    yaw_ = yaw;
    pitch_ = pitch;
    roll_ = roll;
}

double AutoPaths::initYaw(){
    // switch (path_)
    // {
    // case BIG_BOY:
    // {
    //     return 0;
    //     break;
    // }
    // }
    isBlue_ = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    if (isBlue_){
        return -90;
    }
    else{
        return 90;
    }
}

Point AutoPaths::initPos(){
    isBlue_ = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    switch (actions_[0])
    {
    case PRELOADED_CONE_MID:
    {
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_)
        {
            if (left_)
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
            if (left_)
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
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_)
        {
            if (left_)
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
            if (left_)
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
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_)
        {
            if (left_){
                y = FieldConstants::TOP_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else{
                y = FieldConstants::BOTTOM_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else
        {
            if (left_)
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
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_)
        {
            if (left_)
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
            if (left_)
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
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_){
            if (left_)
            {
                y = FieldConstants::TOP_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
            else
            {
                y = FieldConstants::BOTTOM_MIDDLE_CONE_Y - SwerveConstants::CLAW_MID_OFFSET;
            }
        }
        else{
            if (left_)
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
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);

        double y;
        if (isBlue_)
        {
            if (left_)
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
            if (left_)
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
        //double x = FieldConstants::getPos(FieldConstants::AUTO_DOCK_X, isBlue_);
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        return {x, y};
    }
    case TAXI_DOCK_DUMB:
    {
        double y = FieldConstants::AUTO_DOCK_Y;
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        return {x, y};
    }
    case NO_TAXI_DOCK_DUMB:
    {
        double y = FieldConstants::AUTO_DOCK_Y;
        double x = FieldConstants::getPos(FieldConstants::SCORING_X, isBlue_);
        return {x, y};
    }
    case NOTHING: {return {0, 0};} //No initial position
    case DRIVE_BACK_DUMB: {return {0, 0};}
    default: {return {0, 0};}
    }
}

int AutoPaths::pointNum()
{
    return pointNum_;
}

void AutoPaths::setMirrored(bool mirrored)
{
    left_ = mirrored;
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

void AutoPaths::generateXTraj(double pos, double setPos, double vel)
{
    if (slowTraj_){
        xSlowTraj_.generateTrajectory(pos, setPos, vel);
    }
    else{
        xTraj_.generateTrajectory(pos, setPos, vel);
    }
}
void AutoPaths::generateYTraj(double pos, double setPos, double vel){
    if (slowTraj_){
        ySlowTraj_.generateTrajectory(pos, setPos, vel);
    }
    else{
        yTraj_.generateTrajectory(pos, setPos, vel);
    }
}

Pose1D AutoPaths::getXProfile(){
    if (slowTraj_){
        return xSlowTraj_.getProfile();
    }
    else{
        return xTraj_.getProfile();
    }
}
Pose1D AutoPaths::getYProfile(){
    if (slowTraj_){
        return ySlowTraj_.getProfile();
    }
    else{
        return yTraj_.getProfile();
    }
}