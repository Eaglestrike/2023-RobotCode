// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Helpers/GeometryHelper.h"

#include "Controller/ControllerMap.h"

using namespace Actions;

double absDiff(double x, double y){
    return std::abs(x - y);
}

Robot::Robot(): autoPaths_(&swerveDrive_, &arm_){
    AddPeriodic(
        [&]
        {
            double yaw = navx_->GetYaw() - yawOffset_/* + swerveDrive_.getYawTagOffset()*/;
            yaw = GeometryHelper::getPrincipalAng2Deg(yaw);
            frc::SmartDashboard::PutNumber("yaw", yaw);
            frc::SmartDashboard::PutBoolean("navx alive", navx_->IsConnected());
            frc::SmartDashboard::PutBoolean("Data Stale", socketClient_.IsStale());
            frc::SmartDashboard::PutBoolean("Camera Connection", socketClient_.HasConn());

            double ang = (yaw) * M_PI / 180.0;  // Radians
            double pitch = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetPitch() + SwerveConstants::PITCHOFFSET); // Degrees [-180, 180]
            double roll = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetRoll() + SwerveConstants::ROLLOFFSET);    // Degrees [-180, 180]
            double tilt = pitch * sin(ang) - roll * cos(ang); //Field-oriented tilt
            frc::SmartDashboard::PutNumber("Tilt", tilt);
            frc::SmartDashboard::PutNumber("Pitch", pitch);
            frc::SmartDashboard::PutNumber("Roll", roll);

            std::vector<double> data = socketClient_.GetData();
            swerveDrive_.periodic(yaw, tilt, data);

            arm_.periodic();
            cubeIntake_.Periodic();
            arm_.updateIntakeStates(cubeIntake_.getState() == PneumaticsIntake::DEPLOYED, false);

            if (frc::DriverStation::IsAutonomous() && frc::DriverStation::IsEnabled()){
                autoPaths_.periodic();
                autoPaths_.setGyros(yaw, navx_->GetPitch(), navx_->GetRoll());
            }
            else if (frc::DriverStation::IsTeleop()){
                bool armMoving = (arm_.getState() != TwoJointArm::STOPPED && arm_.getState() != TwoJointArm::HOLDING_POS);
                bool armOut = arm_.isArmOut();

                //Panic if arm is out or moving
                swerveDrive_.setPanic((armMoving || armOut));
                
                //Inch/slow down robot
                swerveDrive_.inch(controls_.getPOVDown(INCH_UP),
                                  controls_.getPOVDown(INCH_DOWN),
                                  controls_.getPOVDown(INCH_LEFT),
                                  controls_.getPOVDown(INCH_RIGHT),
                                  controls_.getPressed(SLOW_MODE));
        
                //Pass controller swerve data to swerve
                double xStrafe = controls_.getWithDeadContinuous(XSTRAFE, 0.07);
                double yStrafe = -controls_.getWithDeadContinuous(YSTRAFE, 0.07);
                double rotation = controls_.getWithDeadContinuous(ROTATION, 0.07);
                swerveDrive_.setTarget(xStrafe, yStrafe, rotation);

                //Shift odometry
                double lineupTrimX = controls_.getValueOnce(ControllerMapData::GET_TRIM_X, 0.0);
                if(lineupTrimX != 0.0){
                    swerveDrive_.trim(0.0, lineupTrimX);
                }

                //Periodic call
                swerveDrive_.teleopPeriodic(controls_.getPressed(SCORE),
                                            arm_.isForward(),
                                            scoringLevel_,
                                            controls_.getPressed(LOCK_WHEELS),
                                            controls_.getPressed(AUTO_BALANCE));
            }
        },
        5_ms, 2_ms);
}

void Robot::RobotInit(){
    socketClient_.Init();
    arm_.zeroArmsToAutoStow(true);
    cubeGrabber_.Stop();

    auto1Chooser_.AddOption("Preloaded Cone Mid", AutoPaths::PRELOADED_CONE_MID);
    auto1Chooser_.SetDefaultOption("Preloaded Cone High", AutoPaths::PRELOADED_CONE_HIGH);
    auto1Chooser_.AddOption("Preloaded Cone High Middle", AutoPaths::PRELOADED_CONE_HIGH_MIDDLE);
    auto1Chooser_.AddOption("Preloaded Cone Mid Middle", AutoPaths::PRELOADED_CONE_MID_MIDDLE);
    auto1Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto1Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto1Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto1Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    auto1Chooser_.AddOption("No Taxi Dock Dumb", AutoPaths::NO_TAXI_DOCK_DUMB);
    auto1Chooser_.AddOption("Rotate Only Rotate", AutoPaths::ROTATE_ONLY_ROTATE);
    frc::SmartDashboard::PutData("First Auto Stage", &auto1Chooser_);

    auto2Chooser_.SetDefaultOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    auto2Chooser_.AddOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    auto2Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto2Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto2Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto2Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    auto2Chooser_.AddOption("No Taxi Dock Dumb", AutoPaths::NO_TAXI_DOCK_DUMB);
    auto2Chooser_.AddOption("Rotate Only Rotate", AutoPaths::ROTATE_ONLY_ROTATE);
    frc::SmartDashboard::PutData("Second Auto Stage", &auto2Chooser_);

    auto3Chooser_.AddOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    auto3Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    auto3Chooser_.SetDefaultOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    auto3Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    auto3Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto3Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto3Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto3Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    auto3Chooser_.AddOption("No Taxi Dock Dumb", AutoPaths::NO_TAXI_DOCK_DUMB);
    auto3Chooser_.AddOption("Rotate Only Rotate", AutoPaths::ROTATE_ONLY_ROTATE);
    frc::SmartDashboard::PutData("Third Auto Stage", &auto3Chooser_);

    auto4Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    auto4Chooser_.AddOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    auto4Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    auto4Chooser_.SetDefaultOption("Nothing", AutoPaths::NOTHING);
    auto4Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto4Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto4Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    auto4Chooser_.AddOption("No Taxi Dock Dumb", AutoPaths::NO_TAXI_DOCK_DUMB);
    frc::SmartDashboard::PutData("Fourth Auto Stage", &auto4Chooser_);

    sideChooser_.SetDefaultOption("Right", false);
    sideChooser_.AddOption("Left", true);
    frc::SmartDashboard::PutData("Auto Side", &sideChooser_);

    frc::SmartDashboard::PutBoolean("Slow Trajectory", false);

    cubeIntaking_ = false;
    armsZeroed_ = false;
    scoringLevel_ = 1;
    coneGrabTimerStarted_ = false;
    coneGrabTimerStartTime_ = 0;

    try{
        navx_ = new AHRS(frc::SerialPort::kUSB);
    }
    catch (const std::exception &e){
        std::cout << e.what() << std::endl;
    }
    navx_->ZeroYaw();
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic(){
    frc::SmartDashboard::PutBoolean("Pos Known", !arm_.posUnknown());

    frc::SmartDashboard::PutBoolean("Intaking Cube", cubeIntaking_);
    frc::SmartDashboard::PutBoolean("ESTOPPED", arm_.isEStopped());
    frc::SmartDashboard::PutBoolean("Arms Zeroed", armsZeroed_);

    frc::SmartDashboard::PutString("Arm State", arm_.getStateString());
    frc::SmartDashboard::PutString("Arm Pos", arm_.getPosString());
    frc::SmartDashboard::PutString("Arm Set Pos", arm_.getSetPosString());

    frc::SmartDashboard::PutNumber("x", swerveDrive_.getX());
    frc::SmartDashboard::PutNumber("y", swerveDrive_.getY());
    frc::SmartDashboard::PutNumber("Theta", arm_.getTheta());
    frc::SmartDashboard::PutNumber("Phi", arm_.getPhi());

    frc::SmartDashboard::PutNumber("Scoring Pos", swerveDrive_.getScoringPos());

    //Shift arm
    double armInch = controls_.getValueOnce(ControllerMapData::GET_INCH_ARM_Y, 0.0);
    if(armInch != 0.0){  
        arm_.inchArm(armInch);
    }
    
    //Field orient
    bool isBlue = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    double forward = isBlue? 1.0 : -1.0;
    if (controls_.getPressed(FIELD_ORIENT)){
        navx_->ZeroYaw();
        yawOffset_ = forward * 90.0;
        std::cout<<"ZEROED"<<std::endl;
    }

    //Zero Arms
    if (controls_.getPressed(ZERO_ARMS_1) && controls_.getTriggerDown(ZERO_ARMS_2) && controls_.getTriggerDown(ZERO_ARMS_3)){
        arm_.zeroArms();
        armsZeroed_ = true;
    }

}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit(){
    // arm_.stop();
    arm_.reset();
    arm_.setForward(true);
    // arm_.resetIntaking();

    if (!armsZeroed_){
        arm_.zeroArmsToAutoStow(true);
        armsZeroed_ = true;
    }
    else{
        arm_.zeroArmsToAutoStow(false);
        if(arm_.isArmOut()){
            auto_disable_ = true;
            std::cout<<"RESET ARM ABORT"<<std::endl;
            return;
        }
        else{
            auto_disable_ = false;
        }
    }

    AutoPaths::Path action1 = auto1Chooser_.GetSelected();
    AutoPaths::Path action2 = auto2Chooser_.GetSelected();
    AutoPaths::Path action3 = auto3Chooser_.GetSelected();
    AutoPaths::Path action4 = auto4Chooser_.GetSelected();
    bool slow = frc::SmartDashboard::GetBoolean("Slow Trajectory", true);
    autoPaths_.setMirrored(sideChooser_.GetSelected());

    // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    // fmt::print("Auto selected: {}\n", m_autoSelected);
    autoPaths_.setActions(action1, action2, action3, action4, slow);

    swerveDrive_.reset();

    navx_->ZeroYaw();
    yawOffset_ = autoPaths_.initYaw();

    Point startXY = autoPaths_.initPos();
    //Set initial position to be current position for dumb paths
    if (abs(swerveDrive_.getX() - startXY.getX()) > 1 || abs(swerveDrive_.getY() - startXY.getY()) > 1)
    {
        // frc::SmartDashboard::PutBoolean("F", true); 
        swerveDrive_.setPos(startXY);
    }

    autoPaths_.startTimer();
    autoPaths_.startAutoTimer();
}

void Robot::AutonomousPeriodic(){
    if(auto_disable_){
        return;
    }

    bool clawOpen = autoPaths_.getClawOpen();
    double wheelSpeed = autoPaths_.getWheelSpeed();
    TwoJointArmProfiles::Positions armPosition = autoPaths_.getArmPosition();
    bool forward = autoPaths_.getForward();

    bool cubeIntakeNeededDown = arm_.cubeIntakeNeededDown();
    cubeIntaking_ = autoPaths_.cubeIntaking();

    if (cubeIntaking_){
        cubeIntakeNeededDown = true;
        if (arm_.isForward()) {
            if (arm_.isArmOut()){
                arm_.toggleForwardExtendedToCubeIntake();
            }
            else if (arm_.getPosition() != TwoJointArmProfiles::STOWED){
                arm_.setPosTo(TwoJointArmProfiles::STOWED);
            }
            else{
                arm_.toggleForward();
            }
        }
        else{
            if (arm_.getPosition() != TwoJointArmProfiles::STOWED && arm_.getPosition() != TwoJointArmProfiles::CUBE_INTAKE){
                armPosition = TwoJointArmProfiles::STOWED;
            }
            else{
                cubeIntakeNeededDown = true;
                armPosition = TwoJointArmProfiles::CUBE_INTAKE;
                wheelSpeed = ClawConstants::INTAKING_SPEED;
                clawOpen = true;
            }
        }
    }
    else if (forward && arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE){
        if (arm_.isArmOut()) {
            arm_.specialSetPosTo(arm_.getPosition());
        }
        else{
            arm_.setPosTo(TwoJointArmProfiles::STOWED); // NEUTRAL STOW
        }
    }

    if (arm_.isForward() != forward)
    {
        arm_.toggleForward();
    }

    arm_.setPosTo(armPosition);

    arm_.setClawWheels(wheelSpeed);
    arm_.setClaw(clawOpen);

    if (cubeIntakeNeededDown){
        cubeIntake_.Deploy();
        if (cubeIntaking_){
            cubeIntake_.setRollerMode(PneumaticsIntake::INTAKE);
        }
        else{
            cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
        }
    }
    else{
        cubeIntake_.Stow();
        cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
    }
}

void Robot::TeleopInit(){
    // frc::SmartDashboard::PutNumber("Set Theta", 0);
    // frc::SmartDashboard::PutNumber("Set Phi", 0);
    // frc::SmartDashboard::PutNumber("Swerve Volts", 0);

    cubeIntaking_ = false;
    PCM.EnableDigital();

    arm_.checkPos();
    arm_.stop();
    // arm_.resetIntaking();

    cubeGrabber_.OuttakeSlow();
}

void Robot::TeleopPeriodic(){
    bool isBlue = frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue;
    double forward = isBlue? 1.0 : -1.0;

    swerveDrive_.setScoringPos(controls_.getValue(ControllerMapData::SCORING_POS, -1));
    int level = controls_.getValue(ControllerMapData::GET_LEVEL, -1);
    if (level != -1){
        scoringLevel_ = level;
    }

    bool cubeIntakeNeededDown = arm_.cubeIntakeNeededDown();

    if (controls_.getPressed(AUTO_BALANCE)){
        // ang, pitch, roll
        // 0,  - ,  0
        // 90, 0, -
        // 180, +, 0
        // 270, 0, +
        double yaw = (navx_->GetYaw() - yawOffset_/* + swerveDrive_.getYawTagOffset()*/);
        yaw=GeometryHelper::getPrincipalAng2Deg(yaw);
        double ang = (yaw)*M_PI / 180.0;                                                                       // Radians
        double pitch = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetPitch() + SwerveConstants::PITCHOFFSET); // Degrees
        double roll = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetRoll() + SwerveConstants::ROLLOFFSET);    // Degrees
        double tilt = pitch * sin(ang) - roll * cos(ang);
        if (abs(tilt) < SwerveConstants::AUTODEADANGLE){
            swerveDrive_.lockWheels();
        }
        else{
            double output = -SwerveConstants::AUTOKTILT * tilt;
            swerveDrive_.drive({output, 0}, 0);
        }
    }

    if (controls_.getTriggerDown(MANUAL_CONTROL)){
        double thetaVel = controls_.getRawAxis(MANUAL_THETA);
        double phiVel = controls_.getRawAxis(MANUAL_PHI);
        thetaVel *= abs(thetaVel); //Squared Control -> contact Caleb bc he doesn't know the name, used for driver experience
        phiVel *= abs(phiVel);
        arm_.manualControl(thetaVel, phiVel, true);
        cubeIntaking_ = false;
    }
    else if (controls_.getPressed(GRAVITY)){
        double thetaVel = controls_.getRawAxis(MANUAL_THETA_2);
        thetaVel *= abs(thetaVel); //Squared control thing
        arm_.manualControl(thetaVel, 0, false);
    }
    else if (controls_.getPressed(SCORE) && armsZeroed_){ //Arm logic for autolineup
        Point scoringPos = swerveDrive_.checkScoringPos(scoringLevel_);
        double scoreX = scoringPos.getX();
        double scoreY = scoringPos.getY();
        if (scoreX == 0 && scoreY == 0){  // COULDO get a better flag thing
            // Do nothing? if no valid scoring position
        }
        else{
            cubeIntaking_ = false;

            double wantedYaw;
            bool playerStation;

            double yaw = navx_->GetYaw() - yawOffset_/* + swerveDrive_.getYawTagOffset()*/;
            yaw = GeometryHelper::getPrincipalAng2Deg(yaw);

            wantedYaw = yaw > 0.0 ? 90.0 : -90.0;

            if (swerveDrive_.getScoringPos() == 9){
                wantedYaw += 5;
            }
            
            playerStation = FieldConstants::onPlayerStationHalf(isBlue, scoringPos.getX());

            double yawError = GeometryHelper::getAngDiffDeg(wantedYaw, yaw);
            if (playerStation){
                arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
                double maxPos = FieldConstants::getPos(FieldConstants::PLAYER_STATION_X, isBlue);//don't extend too close to playerstation
                maxPos += forward * (TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][0] + 0.6);
                if ((forward * (maxPos - swerveDrive_.getX())) > 0){ //If bot is less than max
                    if(arm_.setToForward()){
                        if (controls_.getPressed(RAM_PLAYER_STATION)){
                            arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                        }
                        else if (controls_.getPressed(GO_MID)){
                            arm_.setPosTo(TwoJointArmProfiles::MID);
                        }
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS){ //Default player station
                            arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                        }
                        else if (arm_.getPosition() != TwoJointArmProfiles::RAMMING_PLAYER_STATION && arm_.getPosition() != TwoJointArmProfiles::MID){
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                }
            }
            // if nearby, attempt scoring process
            else if(abs(yawError) < 15 && (absDiff(swerveDrive_.getX(), scoringPos.getX()) < 1.5) && (abs(swerveDrive_.getY() - scoringPos.getY()) < 2)){
                std::cout<<"arm out"<<std::endl;
                int scoringPos = swerveDrive_.getScoringPos();
                TwoJointArmProfiles::Positions position = TwoJointArmProfiles::STOWED;
                switch (scoringLevel_){
                    case 0: // Reset, nothing
                        break;
                    case 1: // Low
                        if(arm_.setToForward()){
                            bool atGroundPos = (abs(arm_.getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
                            // arm_.setPosTo(TwoJointArmProfiles::GROUND);
                            if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS){
                                arm_.setJointPath(0, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
                            }
                            else if (arm_.getPosition() != TwoJointArmProfiles::STOWED && !atGroundPos && (!arm_.isArmMoving())){
                                arm_.setPosTo(TwoJointArmProfiles::STOWED);
                            }
                        }
                        break;
                    case 2: //Mid
                        if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8){
                            position = TwoJointArmProfiles::CUBE_MID;
                        }
                        else{
                            position = TwoJointArmProfiles::MID;
                        }
                        if(arm_.setToForward()){
                            arm_.setPosTo(position);
                        }
                        break;
                    case 3: //High
                        if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8){
                            position = TwoJointArmProfiles::CUBE_HIGH;
                        }
                        else{
                            position = TwoJointArmProfiles::HIGH;
                        }
                        if (arm_.setToForward()){
                            arm_.setPosTo(position);
                        }
                        break;
                }
            }
        }
    }
    else if (armsZeroed_){
        if (arm_.getState() == TwoJointArm::MANUAL){ //Check if in manual (somehow)
            arm_.stop();
        }

        if (controls_.getPressed(RAM_PLAYER_STATION)){
            if(arm_.setToForward()){
                if (!cubeIntaking_){
                    arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                }
            }   
        }
        else if (controls_.getPressed(GO_MID)){
            if(arm_.setToForward()){
                if (!cubeIntaking_){ //Don't double extend
                    if (FieldConstants::onPlayerStationHalf(isBlue, swerveDrive_.getX())){
                        arm_.setPosTo(TwoJointArmProfiles::MID);
                    }
                    else{
                        int scoringPos = swerveDrive_.getScoringPos();
                        if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8){ //Sort cube/cone height
                            arm_.setPosTo(TwoJointArmProfiles::CUBE_MID);
                        }
                        else{
                            arm_.setPosTo(TwoJointArmProfiles::MID);
                        }
                    }
                }
            }  
        }
        else if (controls_.getPressed(STOW) && !cubeIntaking_){
            arm_.setPosTo(TwoJointArmProfiles::STOWED);
        }
        else if (controls_.getPressed(GO_HIGH)){
            if(arm_.setToForward()){
                if (!cubeIntaking_){
                    int scoringPos = swerveDrive_.getScoringPos();
                    if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8)
                    {
                        arm_.setPosTo(TwoJointArmProfiles::CUBE_HIGH);
                    }
                    else
                    {
                        arm_.setPosTo(TwoJointArmProfiles::HIGH);
                    }
                }
            }
        }
        else if (controls_.getPOVDown(EXIT_INTAKE)){
            if (arm_.getState() == TwoJointArm::HOLDING_POS){
                if (cubeIntaking_)
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                    // arm_.setClawWheels(0);
                    //  arm_.setClaw(false);
                }
                cubeIntaking_ = !cubeIntaking_;
            }
        }
        else if (controls_.getPressed(FLIP_ARM) && !cubeIntaking_){ //Don't flip when intaking (specialize not forward)
            arm_.toggleForward();
        }
        else
        {
            arm_.setEStopped(false);
        }
    }
    else{
        arm_.stop(); // manual arm or auto lineup not pressed and arms not zeroed
    }

    if (controls_.getPressed(STOP_EVERYTHING)){
        arm_.stop();
        // arm_.resetIntaking();
        cubeIntaking_ = false;
        arm_.setEStopped(true);
    }
    else if (cubeIntaking_ && armsZeroed_){
        if (arm_.getState() == TwoJointArm::STOPPED || arm_.getState() == TwoJointArm::HOMING){
            cubeIntaking_ = false;
        }
        else{
            cubeIntakeNeededDown = true;
            if (arm_.isForward()){
                if (arm_.isArmOut()){
                    arm_.toggleForwardExtendedToCubeIntake();
                }
                else if (arm_.getPosition() != TwoJointArmProfiles::STOWED){
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                }
                else{
                    arm_.toggleForward(); // NEUTRAL STOW
                }
            }
            else{
                if(arm_.getPosition() != TwoJointArmProfiles::CUBE_INTAKE || arm_.getState() != TwoJointArm::HOLDING_POS){
                    arm_.setPosTo(TwoJointArmProfiles::CUBE_INTAKE);
                }
            }

            arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
            arm_.setClaw(true);
        }
    }
    else{
        coneGrabTimerStarted_ = false;
        grabbedCone_ = false;
    }
    // else if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && arm_.getState() != TwoJointArm::MANUAL)
    // {
    //     arm_.setPosTo(TwoJointArmProfiles::STOWED);
    // }

    if (controls_.getPressedOnce(TOGGLE_CLAW) && !cubeIntaking_){
        arm_.setClaw(!arm_.getClawOpen());
    }

    if (controls_.getPOVDown(CUBE_INTAKE)){
        if(arm_.setToForward()){
            bool atGroundPos = (abs(arm_.getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
            // arm_.setPosTo(TwoJointArmProfiles::GROUND);
            if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS){
                arm_.setJointPath(0, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
            }
            else if (arm_.getPosition() != TwoJointArmProfiles::STOWED && !atGroundPos && (!arm_.isArmMoving())){
                arm_.setPosTo(TwoJointArmProfiles::STOWED);
            }
        }
    }
    
    if (controls_.getPressedOnce(INTAKE) && !cubeIntaking_){ //Toggle claw intake
        if (arm_.getClawWheelSpeed() == ClawConstants::INTAKING_SPEED){
            arm_.setClawWheels(0);
        }
        else{
            arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
        }
    }
    else if (controls_.getPressedOnce(OUTAKE) && !cubeIntaking_) {//Toggle claw outtake
        if (arm_.getClawWheelSpeed() == ClawConstants::OUTAKING_SPEED){
            arm_.setClawWheels(0);
        }
        else{
            arm_.setClawWheels(ClawConstants::OUTAKING_SPEED);
        }           
    }

    
    if(controls_.getPOVDownOnce(CUTOUT_OUTAKE)){ //Set default to be outtake slow
        if(cubeGrabber_.getState() == CubeGrabber::OUTTAKING){
            cubeGrabber_.OuttakeSlow();
        }
        else{
            cubeGrabber_.Outtake();
        }
    }
    else if(controls_.getPOVDown(CUTOUT_INTAKE)){
        cubeGrabber_.Intake();
    }

    if (cubeIntakeNeededDown){
        cubeIntake_.Deploy();
        if (cubeIntaking_){
            cubeIntake_.setRollerMode(PneumaticsIntake::INTAKE);
        }
        else{
            cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
        }
    }
    else{
        cubeIntake_.Stow();
        cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
    }

    frc::SmartDashboard::PutBoolean("Cube Intake Down", cubeIntakeNeededDown);
}

void Robot::DisabledInit(){
    arm_.stop();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
    PCM.Disable();
}

void Robot::DisabledPeriodic(){
    frc::SmartDashboard::PutNumber("Theta", arm_.getTheta());
    swerveDrive_.reset();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
    arm_.checkPos();
    cubeGrabber_.Stop();

    // Calling all the pressed functions so that they don't buffer
    controls_.stopBuffer();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
