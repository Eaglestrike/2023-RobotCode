// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "Helpers/GeometryHelper.h"

#include "Controller/ControllerMap.h"

using namespace Actions;

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

            double ang = (yaw)*M_PI / 180.0;                                                                       // Radians
            double pitch = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetPitch() + SwerveConstants::PITCHOFFSET); // Degrees [-180, 180]
            double roll = GeometryHelper::getPrincipalAng2Deg((double)navx_->GetRoll() + SwerveConstants::ROLLOFFSET);    // Degrees [-180, 180]
            double tilt = pitch * sin(ang) - roll * cos(ang); //Field-oriented tilt
            frc::SmartDashboard::PutNumber("Tilt", tilt);
            frc::SmartDashboard::PutNumber("Pitch", pitch);
            frc::SmartDashboard::PutNumber("Roll", roll);
            // frc::SmartDashboard::PutNumber("Pitch Raw", navx_->GetPitch());
            // frc::SmartDashboard::PutNumber("Roll Raw", navx_->GetRoll());

            std::vector<double> data = socketClient_.GetData();
            swerveDrive_.periodic(yaw, tilt, data);

            arm_.periodic();
            cubeIntake_.Periodic();
            arm_.updateIntakeStates(cubeIntake_.getState() == PneumaticsIntake::DEPLOYED, false); // TODO here for cone intake

            if (frc::DriverStation::IsAutonomous() && frc::DriverStation::IsEnabled())
            {
                autoPaths_.periodic();
                autoPaths_.setGyros(yaw, navx_->GetPitch(), navx_->GetRoll());
            }
            else if (frc::DriverStation::IsTeleop())
            {
                bool armMoving = (arm_.getState() != TwoJointArm::STOPPED && arm_.getState() != TwoJointArm::HOLDING_POS);
                // bool armOut = (arm_.getPosition() != TwoJointArmProfiles::STOWED /* && arm_.getPosition() != TwoJointArmProfiles::CONE_INTAKE*/ && arm_.getPosition() != TwoJointArmProfiles::CUBE_INTAKE && arm_.getPosition() != TwoJointArmProfiles::GROUND);
                bool armOut = (arm_.getPosition() == TwoJointArmProfiles::MID || arm_.getPosition() == TwoJointArmProfiles::HIGH || arm_.getPosition() == TwoJointArmProfiles::CUBE_MID || arm_.getPosition() == TwoJointArmProfiles::CUBE_HIGH); // TODO make this based on xy position

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
                double lineupTrimY = controls_.getValueOnce(ControllerMapData::GET_TRIM_Y, 0.0);
                swerveDrive_.trim(lineupTrimX, lineupTrimY);
                swerveDrive_.teleopPeriodic(controls_.getPressed(SCORE),
                                            arm_.isForward(),
                                            scoringLevel_,
                                            controls_.getPressed(LOCK_WHEELS),
                                            controls_.getPressed(AUTO_BALANCE));
            }

            // if (frc::DriverStation::IsEnabled())
            // {
            //     arm_.periodic();
            //     bool armMoving = (arm_.getState() != TwoJointArm::STOPPED && arm_.getState() != TwoJointArm::HOLDING_POS);
            //     swerveDrive_.periodic(yaw, controls_, arm_.isForward(), armMoving);
            // }
        },
        5_ms, 2_ms);
}

void Robot::RobotInit()
{

    // frc::SmartDashboard::PutBoolean("Sending it Fast", false);
    // frc::SmartDashboard::PutBoolean("Sending it Medium", false);
    // frc::SmartDashboard::PutBoolean("Balanced", false);
    socketClient_.Init();
    arm_.zeroArmsToAutoStow();
    cubeGrabber_.Stop();

    auto1Chooser_.AddOption("Preloaded Cone Mid", AutoPaths::PRELOADED_CONE_MID);
    // auto1Chooser_.AddOption("Preloaded Cube Mid", AutoPaths::PRELOADED_CUBE_MID);
    auto1Chooser_.SetDefaultOption("Preloaded Cone High", AutoPaths::PRELOADED_CONE_HIGH);
    // auto1Chooser_.AddOption("Preloaded Cube High", AutoPaths::PRELOADED_CUBE_HIGH);
    auto1Chooser_.AddOption("Preloaded Cone High Middle", AutoPaths::PRELOADED_CONE_HIGH_MIDDLE);
    auto1Chooser_.AddOption("Preloaded Cone Mid Middle", AutoPaths::PRELOADED_CONE_MID_MIDDLE);
    // auto1Chooser_.AddOption("First Cone Mid", AutoPaths::FIRST_CONE_MID);
    // auto1Chooser_.AddOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    // auto1Chooser_.AddOption("First Cone Dock", AutoPaths::FIRST_CONE_DOCK);
    // auto1Chooser_.AddOption("First Cube Dock", AutoPaths::FIRST_CUBE_DOCK);
    // auto1Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    // auto1Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    // auto1Chooser_.AddOption("Second Cone Dock", AutoPaths::SECOND_CONE_DOCK);
    // auto1Chooser_.AddOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    // auto1Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    // auto1Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto1Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto1Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto1Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto1Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    frc::SmartDashboard::PutData("First Auto Stage", &auto1Chooser_);

    // auto2Chooser_.AddOption("Preloaded Cone Mid", AutoPaths::PRELOADED_CONE_MID);
    // auto2Chooser_.AddOption("Preloaded Cube Mid", AutoPaths::PRELOADED_CUBE_MID);
    // auto2Chooser_.AddOption("Preloaded Cone High", AutoPaths::PRELOADED_CONE_HIGH);
    // auto2Chooser_.AddOption("Preloaded Cube High", AutoPaths::PRELOADED_CUBE_HIGH);
    // auto2Chooser_.AddOption("Preloaded Cone High Middle", AutoPaths::PRELOADED_CONE_HIGH_MIDDLE);
    // auto2Chooser_.AddOption("Preloaded Cone Mid Middle", AutoPaths::PRELOADED_CONE_MID_MIDDLE);
    // auto2Chooser_.AddOption("First Cone Mid", AutoPaths::FIRST_CONE_MID);
    auto2Chooser_.SetDefaultOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    // auto2Chooser_.AddOption("First Cone Dock", AutoPaths::FIRST_CONE_DOCK);
    // auto2Chooser_.AddOption("First Cube Dock", AutoPaths::FIRST_CUBE_DOCK);
    // auto2Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    // auto2Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    // auto2Chooser_.AddOption("Second Cone Dock", AutoPaths::SECOND_CONE_DOCK);
    // auto2Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    auto2Chooser_.AddOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    // auto2Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto2Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto2Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto2Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto2Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    frc::SmartDashboard::PutData("Second Auto Stage", &auto2Chooser_);

    // auto3Chooser_.AddOption("Preloaded Cone Mid", AutoPaths::PRELOADED_CONE_MID);
    // auto3Chooser_.AddOption("Preloaded Cube Mid", AutoPaths::PRELOADED_CUBE_MID);
    // auto3Chooser_.AddOption("Preloaded Cone High", AutoPaths::PRELOADED_CONE_HIGH);
    // auto3Chooser_.AddOption("Preloaded Cube High", AutoPaths::PRELOADED_CUBE_HIGH);
    // auto3Chooser_.AddOption("Preloaded Cone High Middle", AutoPaths::PRELOADED_CONE_HIGH_MIDDLE);
    // auto3Chooser_.AddOption("Preloaded Cone Mid Middle", AutoPaths::PRELOADED_CONE_MID_MIDDLE);
    // auto3Chooser_.AddOption("First Cone Mid", AutoPaths::FIRST_CONE_MID);
    auto3Chooser_.AddOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    // auto3Chooser_.AddOption("First Cone Dock", AutoPaths::FIRST_CONE_DOCK);
    // auto3Chooser_.AddOption("First Cube Dock", AutoPaths::FIRST_CUBE_DOCK);
    // auto3Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    auto3Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    // auto3Chooser_.AddOption("Second Cone Dock", AutoPaths::SECOND_CONE_DOCK);
    auto3Chooser_.SetDefaultOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    auto3Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    // auto3Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto3Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto3Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto3Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto3Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    frc::SmartDashboard::PutData("Third Auto Stage", &auto3Chooser_);

    // auto4Chooser_.AddOption("Preloaded Cone Mid", AutoPaths::PRELOADED_CONE_MID);
    // auto4Chooser_.AddOption("Preloaded Cube Mid", AutoPaths::PRELOADED_CUBE_MID);
    // auto4Chooser_.AddOption("Preloaded Cone High", AutoPaths::PRELOADED_CONE_HIGH);
    // auto4Chooser_.AddOption("Preloaded Cube High", AutoPaths::PRELOADED_CUBE_HIGH);
    // auto4Chooser_.AddOption("Preloaded Cone High Middle", AutoPaths::PRELOADED_CONE_HIGH_MIDDLE);
    // auto4Chooser_.AddOption("Preloaded Cone Mid Middle", AutoPaths::PRELOADED_CONE_MID_MIDDLE);
    // auto4Chooser_.AddOption("First Cone Mid", AutoPaths::FIRST_CONE_MID);
    // auto4Chooser_.AddOption("First Cube High", AutoPaths::FIRST_CUBE_HIGH);
    // auto4Chooser_.AddOption("First Cone Dock", AutoPaths::FIRST_CONE_DOCK);
    // auto4Chooser_.AddOption("First Cube Dock", AutoPaths::FIRST_CUBE_DOCK);
    // auto4Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    auto4Chooser_.AddOption("Second Cube Mid", AutoPaths::SECOND_CUBE_MID);
    // auto4Chooser_.AddOption("Second Cone Dock", AutoPaths::SECOND_CONE_DOCK);
    auto4Chooser_.AddOption("Second Cube Dock", AutoPaths::SECOND_CUBE_DOCK);
    auto4Chooser_.AddOption("Second Cube Grab", AutoPaths::SECOND_CUBE_GRAB);
    // auto4Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto4Chooser_.SetDefaultOption("Nothing", AutoPaths::NOTHING);
    auto4Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto4Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    auto4Chooser_.AddOption("Taxi Dock Dumb", AutoPaths::TAXI_DOCK_DUMB);
    frc::SmartDashboard::PutData("Fourth Auto Stage", &auto4Chooser_);

    sideChooser_.SetDefaultOption("Right", false);
    sideChooser_.AddOption("Left", true);
    frc::SmartDashboard::PutData("Auto Side", &sideChooser_);

    cubeIntaking_ = false;
    coneIntaking_ = false;
    coneIntakeDown_ = false;
    armsZeroed_ = false;
    scoringLevel_ = 1;
    // psType_ = 1;
    coneGrabTimerStarted_ = false;
    coneGrabTimerStartTime_ = 0;
    // frc::SmartDashboard::PutNumber("Test Volts", 3);

    try
    {
        navx_ = new AHRS(frc::SerialPort::kUSB);
    }
    catch (const std::exception &e)
    {
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
void Robot::RobotPeriodic()
{
    // frc::SmartDashboard::PutBoolean("Shoulder Brake", arm_.shoulderBrakeEngaged());
    // frc::SmartDashboard::PutBoolean("Elbow Brake", arm_.elbowBrakeEngaged());
    frc::SmartDashboard::PutBoolean("Forward", arm_.isForward());
    frc::SmartDashboard::PutBoolean("Pos Known", !arm_.posUnknown());
    // frc::SmartDashboard::PutBoolean("Intaking Cone", coneIntaking_);
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
    // frc::SmartDashboard::PutNumber("Theta vel", arm_.getThetaVel());
    // frc::SmartDashboard::PutNumber("Phi vel", arm_.getPhiVel());
    // frc::SmartDashboard::PutNumber("Theta Volts", arm_.getThetaVolts());
    // frc::SmartDashboard::PutNumber("Phi Volts", arm_.getPhiVolts());

    frc::SmartDashboard::PutNumber("Scoring Pos", swerveDrive_.getScoringPos());

    frc::SmartDashboard::PutBoolean("Cutout Intaking", cubeGrabber_.isIntaking());
    frc::SmartDashboard::PutBoolean("Cutout Outaking", cubeGrabber_.isOuttaking());
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
void Robot::AutonomousInit()
{
    // arm_.stop();
    arm_.reset();
    arm_.setForward(true);
    // arm_.resetIntaking();

    if (!armsZeroed_){
        arm_.zeroArmsToAutoStow();
        armsZeroed_ = true;
    }
    else{
        if(arm_.isArmOut()){
            arm_.stop();
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
    autoPaths_.setMirrored(sideChooser_.GetSelected());

    // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    // fmt::print("Auto selected: {}\n", m_autoSelected);
    autoPaths_.setActions(action1, action2, action3, action4);

    swerveDrive_.reset();

    navx_->ZeroYaw();
    yawOffset_ = autoPaths_.initYaw();

    Point startXY = autoPaths_.initPos();
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

    // bool clawOpen = autoPaths_.getClawOpen();
    // double wheelSpeed = autoPaths_.getWheelSpeed();
    // TwoJointArmProfiles::Positions armPosition = autoPaths_.getArmPosition();
    // bool forward = autoPaths_.getForward();

    // bool cubeIntakeNeededDown = arm_.cubeIntakeNeededDown();
    // cubeIntaking_ = autoPaths_.cubeIntaking();
    // coneIntaking_ = autoPaths_.coneIntaking();

    // if (cubeIntaking_){
    //     cubeIntakeNeededDown = true;
    //     if (arm_.isForward()) {
    //         if (arm_.isArmOut()){
    //             arm_.toggleForwardExtendedToCubeIntake();
    //         }
    //         else if (arm_.getPosition() != TwoJointArmProfiles::STOWED){
    //             arm_.setPosTo(TwoJointArmProfiles::STOWED);
    //         }
    //         else{
    //             // arm_.toggleForwardCubeIntake();//NEUTRAL STOW
    //             arm_.toggleForward(); // NEUTRAL STOW
    //         }
    //     }
    //     else
    //     {
    //         if (arm_.getPosition() != TwoJointArmProfiles::STOWED && arm_.getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
    //         {
    //             armPosition = TwoJointArmProfiles::STOWED;
    //         }
    //         else
    //         {
    //             cubeIntakeNeededDown = true;
    //             armPosition = TwoJointArmProfiles::CUBE_INTAKE;
    //             wheelSpeed = ClawConstants::INTAKING_SPEED;
    //             clawOpen = true;
    //         }
    //     }
    // }
    // else if (forward && arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE)
    // {
    //     if (arm_.isArmOut()) {
    //         arm_.specialSetPosTo(arm_.getPosition());
    //     }
    //     else
    //     {
    //         // arm_.toggleForwardCubeIntake(); //NEUTRAL STOW
    //         arm_.setPosTo(TwoJointArmProfiles::STOWED); // NEUTRAL STOW
    //     }
    // }
    // else
    // {
    //     // if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE)
    //     // {
    //     //     armPosition = TwoJointArmProfiles::STOWED;
    //     // }
    // }

    // if (arm_.isForward() != forward)
    // {
    //     arm_.toggleForward();
    // }

    // arm_.setPosTo(armPosition);

    // arm_.setClawWheels(wheelSpeed);
    // arm_.setClaw(clawOpen);

    // if (cubeIntakeNeededDown)
    // {
    //     cubeIntake_.Deploy();
    //     if (cubeIntaking_)
    //     {
    //         cubeIntake_.setRollerMode(PneumaticsIntake::INTAKE);
    //     }
    //     else
    //     {
    //         cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
    //     }
    // }
    // else
    // {
    //     cubeIntake_.Stow();
    //     cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
    // }
}

void Robot::TeleopInit()
{
    // frc::SmartDashboard::PutNumber("Set Theta", 0);
    // frc::SmartDashboard::PutNumber("Set Phi", 0);
    // frc::SmartDashboard::PutNumber("Swerve Volts", 0);

    cubeIntaking_ = false;
    coneIntaking_ = false;
    coneIntakeDown_ = false;
    PCM.EnableDigital();

    arm_.checkPos();
    arm_.stop();
    // arm_.resetIntaking();

    // cubeGrabber_.OuttakeSlow(); TODO DELETE
}

void Robot::TeleopPeriodic()
{
    swerveDrive_.setScoringPos(controls_.getValue(ControllerMapData::SCORING_POS, -1));
    int level = controls_.getValue(ControllerMapData::GET_LEVEL, -1);
    if (level != -1){
        scoringLevel_ = level;
    }

    // if (controls_.checkPSButtons() != -1)
    // {
    //     psType_ = controls_.checkPSButtons();
    // }

    bool cubeIntakeNeededDown = arm_.cubeIntakeNeededDown();
    bool coneIntakeHalfway = false;

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

    if (controls_.getPressed(FIELD_ORIENT))
    {
        navx_->ZeroYaw();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue){
            yawOffset_ = 90;
        }
        else{
            yawOffset_ = -90;
        }
        std::cout<<"ZEROED"<<std::endl;
        // swerveDrive_.resetYawTagOffset();
    }

    if (controls_.getPressed(ZERO_ARMS_1) &&
        controls_.getTriggerDown(ZERO_ARMS_2) &&
        controls_.getTriggerDown(ZERO_ARMS_3))
    {
        arm_.zeroArms();
        armsZeroed_ = true;
    }

    if (controls_.getTriggerDown(MANUAL_CONTROL)){
        double thetaVel = controls_.getRawAxis(MANUAL_THETA);
        thetaVel *= abs(thetaVel); //Squared Control -> contact Caleb bc he doesn't know the name, used for driver experience
        double phiVel = controls_.getRawAxis(MANUAL_PHI);
        phiVel *= abs(phiVel);
        arm_.manualControl(thetaVel, phiVel, true);
        cubeIntaking_ = false;
        coneIntaking_ = false;
        coneIntakeDown_ = false;
    }
    else if (controls_.getPressed(GRAVITY))
    {
        double thetaVel = controls_.getRawAxis(MANUAL_THETA_2);
        thetaVel *= abs(thetaVel); //Squared control thing
        arm_.manualControl(thetaVel, 0, false);
    }
    else if (controls_.getPressed(SCORE) && armsZeroed_)
    {
        Point scoringPos = swerveDrive_.checkScoringPos(scoringLevel_);
        if (scoringPos.getX() == 0 && scoringPos.getY() == 0) // COULDO get a better flag thing
        {
            // Do nothing? if no valid scoring position
        }
        else
        {
            cubeIntaking_ = false;
            coneIntaking_ = false;

            double wantedYaw;
            bool playerStation;
            // if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) FORWARD BASED LINEUP
            // {
            //     if (arm_.isForward())
            //     {
            //         wantedYaw = 90.0;
            //         if (scoringPos.getX() > 6.0)
            //         {
            //             playerStation = true;
            //             wantedYaw *= -1;
            //         }
            //     }
            //     else
            //     {
            //         wantedYaw = -90.0;
            //         if (scoringPos.getX() > 6.0)
            //         {
            //             playerStation = true;
            //             wantedYaw *= -1;
            //         }
            //     }
            // }
            // else
            // {
            //     if (arm_.isForward())
            //     {
            //         wantedYaw = -90.0;
            //         if (scoringPos.getX() < 6.0)
            //         {
            //             playerStation = true;
            //             wantedYaw *= -1;
            //         }
            //     }
            //     else
            //     {
            //         wantedYaw = 90.0;
            //         if (scoringPos.getX() < 6.0)
            //         {
            //             playerStation = true;
            //             wantedYaw *= -1;
            //         }
            //     }
            // }

            double yaw = navx_->GetYaw() - yawOffset_/* + swerveDrive_.getYawTagOffset()*/;
            yaw = GeometryHelper::getPrincipalAng2Deg(yaw);

            wantedYaw = yaw > 0.0 ? 90.0 : -90.0;

            if (swerveDrive_.getScoringPos() == 9){
                wantedYaw += 5;
            }
            
            playerStation = FieldConstants::onPlayerStationHalf(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue, scoringPos.getX());

            double yawError = GeometryHelper::getAngDiffDeg(wantedYaw, yaw);

            //If too far away or not lined up enough
            if (abs(yawError) > 15 || (!playerStation && (abs(swerveDrive_.getX() - scoringPos.getX()) > 1.5 || abs(swerveDrive_.getY() - scoringPos.getY()) > 2)))
            {
                // Do nothing
            }
            else if (playerStation)
            {
                arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
                if (
                    (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue &&
                    swerveDrive_.getX() < FieldConstants::PLAYER_STATION_X.blue - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][0] + 0.6)
                    ||
                    (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed &&
                    swerveDrive_.getX() > FieldConstants::PLAYER_STATION_X.red + TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][0] - 0.6)
                    )
                {
                    // arm_.setPosTo(TwoJointArmProfiles::MID); //FORWARD BASED LINEUP

                    // TwoJointArmProfiles::Positions position = TwoJointArmProfiles::RAMMING_PLAYER_STATION;
                    // if (psType_ == 2)
                    // {
                    //     position = TwoJointArmProfiles::MID;
                    // }

                    if (!arm_.isForward())
                    {
                        /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            cubeIntaking_ = false;
                            arm_.specialSetPosTo(position);
                        }
                        else */
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.toggleForward();
                        }
                        else
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                    else
                    {
                        if (controls_.getPressed(RAM_PLAYER_STATION))
                        {
                            arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                        }
                        else if (controls_.getPressed(GO_MID))
                        {
                            arm_.setPosTo(TwoJointArmProfiles::MID);
                        }
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                        }
                        else if (arm_.getPosition() != TwoJointArmProfiles::RAMMING_PLAYER_STATION && arm_.getPosition() != TwoJointArmProfiles::MID)
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                }
            }

            // if nearby, attempt scoring process
            // original condition: abs(swerveDrive_.getX() - scoringPos.getX()) < 1 && abs(swerveDrive_.getY() - scoringPos.getY()) < 1
            else
            {
                switch (scoringLevel_)
                {
                case 0: // Reset, nothing
                {
                    break;
                }
                case 1: // Low
                {
                    if (!arm_.isForward())
                    {
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.toggleForward();
                        }
                        else
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                    else
                    {
                        bool atGroundPos = (abs(arm_.getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
                        // arm_.setPosTo(TwoJointArmProfiles::GROUND);
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.setJointPath(0, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
                        }
                        else if (arm_.getPosition() != TwoJointArmProfiles::STOWED && !atGroundPos && (!arm_.isArmMoving()))
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                    break;
                }
                case 2: //Mid
                {
                    int scoringPos = swerveDrive_.getScoringPos();
                    TwoJointArmProfiles::Positions position;
                    if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8)
                    {
                        position = TwoJointArmProfiles::CUBE_MID; // FORWARD BASED LINEUP
                        // arm_.setPosTo(TwoJointArmProfiles::CUBE_MID);
                    }
                    else
                    {
                        position = TwoJointArmProfiles::MID;
                        // arm_.setPosTo(TwoJointArmProfiles::MID); // MID THING and FORWARD BASED LINEUP
                    }

                    if (!arm_.isForward())
                    {
                        /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            cubeIntaking_ = false;
                            arm_.specialSetPosTo(position);
                        }
                        else */
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.toggleForward();
                        }
                        else
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                    else
                    {
                        arm_.setPosTo(position);
                    }
                    break;
                }
                case 3: //High
                {
                    int scoringPos = swerveDrive_.getScoringPos();
                    TwoJointArmProfiles::Positions position;
                    if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8){
                        position = TwoJointArmProfiles::CUBE_HIGH;
                        // arm_.setPosTo(TwoJointArmProfiles::CUBE_HIGH);//FORWARD BASED LINEUP
                    }
                    else{
                        position = TwoJointArmProfiles::HIGH;
                        // arm_.setPosTo(TwoJointArmProfiles::HIGH);//FORWARD BASED LINEUP
                    }

                    if (!arm_.isForward())
                    {
                        /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            cubeIntaking_ = false;
                            arm_.specialSetPosTo(position);
                        }
                        else */
                        if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                        {
                            arm_.toggleForward();
                        }
                        else
                        {
                            arm_.setPosTo(TwoJointArmProfiles::STOWED);
                        }
                    }
                    else
                    {
                        arm_.setPosTo(position);
                    }
                    break;
                }
                }
            }
        }
    }
    else if (armsZeroed_)
    {
        // if(controls_.aPressed())
        // {
        //     arm_.goToPos(frc::SmartDashboard::GetNumber("Set Theta", 0), frc::SmartDashboard::GetNumber("Set Phi", 0));
        // }
        // else
        // {
        //     //arm_.stop();
        // }

        if (arm_.getState() == TwoJointArm::MANUAL)
        {
            arm_.stop();
        }

        if (controls_.getPressed(RAM_PLAYER_STATION))
        {
            if (!arm_.isForward())
            {
                /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    cubeIntaking_ = false;
                    arm_.specialSetPosTo(position);
                }
                else */
                if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    arm_.toggleForward();
                }
                else
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                }
            }
            else
            {
                if (!coneIntaking_ && !cubeIntaking_){
                    arm_.setPosTo(TwoJointArmProfiles::RAMMING_PLAYER_STATION);
                }
            }   
        }
        else if (controls_.getPressed(GO_MID))
        {
            if (!arm_.isForward())
            {
                /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    cubeIntaking_ = false;
                    arm_.specialSetPosTo(position);
                }
                else */
                if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    arm_.toggleForward();
                }
                else
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                }
            }
            else
            {
                if (!coneIntaking_ && !cubeIntaking_)
                {
                    if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue && swerveDrive_.getX() > FieldConstants::FIELD_LENGTH / 2)
                    {
                        arm_.setPosTo(TwoJointArmProfiles::MID); // MID THING
                    }
                    else if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && swerveDrive_.getX() < FieldConstants::FIELD_LENGTH / 2)
                    {
                        arm_.setPosTo(TwoJointArmProfiles::MID); // MID THING
                    }
                    else
                    {
                        int scoringPos = swerveDrive_.getScoringPos();
                        if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8)
                        {
                            arm_.setPosTo(TwoJointArmProfiles::CUBE_MID);
                        }
                        else
                        {
                            arm_.setPosTo(TwoJointArmProfiles::MID); // MID THING
                        }
                    }
                }
            }  
        }
        else if (controls_.getPressed(STOW)){
            if (!coneIntaking_ && !cubeIntaking_)
            {
                arm_.setPosTo(TwoJointArmProfiles::STOWED);
            }
        }
        else if (controls_.getPressed(GO_HIGH)){
            if (!arm_.isForward())
            {
                /*if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && !arm_.isForward() && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    cubeIntaking_ = false;
                    arm_.specialSetPosTo(position);
                }
                else */
                if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
                {
                    arm_.toggleForward();
                }
                else
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                }
            }
            else
            {
                if (!coneIntaking_ && !cubeIntaking_)
                {
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
        else if (controls_.getPOVDown(EXIT_INTAKE))
        {
            if (arm_.getState() == TwoJointArm::HOLDING_POS)
            {
                if (cubeIntaking_)
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                    // arm_.setClawWheels(0);
                    //  arm_.setClaw(false);
                }
                cubeIntaking_ = !cubeIntaking_;
                coneIntaking_ = false;
            }
        }
        // else if (coneIntakePressed)
        // {
        //     // if (arm_.getState() == TwoJointArm::HOLDING_POS)
        //     // {
        //     //     if (coneIntaking_)
        //     //     {
        //     //         arm_.setPosTo(TwoJointArmProfiles::STOWED);
        //     //         arm_.setClawWheels(0);
        //     //         // arm_.setClaw(false);
        //     //     }
        //     //     coneIntaking_ = !coneIntaking_;
        //     //     cubeIntaking_ = false;
        //     // }

        //     if (arm_.getState() == TwoJointArm::HOLDING_POS && arm_.getPosition() == TwoJointArmProfiles::STOWED)
        //     {
        //         coneIntaking_ = true;
        //     }
        // }
        else if (controls_.getPressed(FLIP_ARM))
        {
            // if (cubeIntaking_ && arm_.getState() == TwoJointArm::HOLDING_POS)//NEUTRAL STOW
            // {
            //     arm_.toggleForwardCubeIntake();
            //     cubeIntaking_ = false;
            // }
            if (cubeIntaking_)
            {
                // arm_.setPosTo(TwoJointArmProfiles::STOWED);
                // return;
            }
            else
            {
                arm_.toggleForward();
            }
        }
        else
        {
            arm_.setEStopped(false);
        }
    }
    else
    {
        // manual arm or auto lineup not pressed and arms not zeroed
        arm_.stop();
    }

    if (controls_.getPressed(STOP_EVERYTHING))
    {
        arm_.stop();
        // arm_.resetIntaking();
        coneIntaking_ = false;
        cubeIntaking_ = false;
        coneIntakeDown_ = false;
        arm_.setEStopped(true);
    }
    else if (cubeIntaking_ && armsZeroed_)
    {
        if (arm_.getState() == TwoJointArm::STOPPED || arm_.getState() == TwoJointArm::HOMING)
        {
            cubeIntaking_ = false;
        }
        else
        {
            cubeIntakeNeededDown = true;
            if (arm_.isForward())
            {
                if (arm_.isArmOut())
                {
                    arm_.toggleForwardExtendedToCubeIntake();
                }
                else if (arm_.getPosition() != TwoJointArmProfiles::STOWED)
                {
                    arm_.setPosTo(TwoJointArmProfiles::STOWED);
                }
                else
                {
                    // arm_.toggleForwardCubeIntake();//NEUTRAL STOW
                    arm_.toggleForward(); // NEUTRAL STOW
                }
            }
            else
            {
                if(arm_.getPosition() != TwoJointArmProfiles::CUBE_INTAKE || arm_.getState() != TwoJointArm::HOLDING_POS)
                {
                    arm_.setPosTo(TwoJointArmProfiles::CUBE_INTAKE);
                }
            }

            arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
            arm_.setClaw(true);
        }
    }
    // else if (coneIntaking_ && armsZeroed_)
    // {
    //     bool atConeIntakePos = (abs(arm_.getTheta() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
    //     if (/*arm_.getState() == TwoJointArm::STOPPED || */ arm_.getState() == TwoJointArm::HOMING)
    //     {
    //         coneIntaking_ = false;
    //     }
    //     else if (arm_.getPosition() != TwoJointArmProfiles::STOWED && !atConeIntakePos /* || arm_.getPosition() != TwoJointArmProfiles::CONE_INTAKE*/)
    //     {
    //         coneIntaking_ = false;
    //     }
    //     else
    //     {
    //         if (!arm_.isForward())
    //         {
    //             if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
    //             {
    //                 arm_.toggleForward();
    //             }
    //             else
    //             {
    //                 arm_.setPosTo(TwoJointArmProfiles::STOWED);
    //             }
    //         }

    //         if (!arm_.isForward())
    //         {
    //         }
    //         else if (!grabbedCone_)
    //         {
    //             intakesNeededDown.getY() = true;
    //             coneIntakeDown_ = false;
    //             if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
    //             {
    //                 // coneIntakeHalfway = true;
    //                 // arm_.setPosTo(TwoJointArmProfiles::CONE_INTAKE);

    //                 intakesNeededDown.getY() = true;
    //                 coneIntakeDown_ = false;
    //                 arm_.setJointPath(arm_.getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
    //                 arm_.setClaw(true);
    //             }
    //             else if ((arm_.getState() == TwoJointArm::STOPPED || arm_.getState() == TwoJointArm::HOLDING_POS) && (abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getTheta() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][2]) > TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD))
    //             {
    //                 arm_.setJointPath(TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][2], TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
    //             }
    //             else if (atConeIntakePos && (arm_.getState() == TwoJointArm::STOPPED || arm_.getState() == TwoJointArm::HOLDING_POS))
    //             {
    //                 if (!coneGrabTimerStarted_)
    //                 {
    //                     coneGrabTimerStartTime_ = timer_.GetFPGATimestamp().value();
    //                     coneGrabTimerStarted_ = true;
    //                 }

    //                 arm_.setClawWheels(ClawConstants::INTAKING_SPEED);

    //                 double time = timer_.GetFPGATimestamp().value() - coneGrabTimerStartTime_;

    //                 coneIntakeHalfway = true;

    //                 if (time > 0.25)
    //                 {
    //                     arm_.setClaw(false);
    //                     // intakesNeededDown.getY() = false;
    //                     // coneIntakeHalfway = false;
    //                     // OUTAKE CONE
    //                 }

    //                 if(time > 0.45)
    //                 {
    //                     intakesNeededDown.getY() = false;
    //                     coneIntakeHalfway = false;
    //                     coneIntakeDown_ = false;
    //                 }

    //                 if (time > 0.6)
    //                 {
    //                     grabbedCone_ = true;
    //                     coneGrabTimerStarted_ = false;
    //                 }
    //             }
    //         }
    //         else
    //         {
    //             arm_.setClawWheels(0);
    //             arm_.setClaw(false);

    //             // if (!coneGrabTimerStarted_)
    //             // {
    //             //     coneGrabTimerStartTime_ = timer_.GetFPGATimestamp().value();
    //             //     coneGrabTimerStarted_ = true;
    //             // }

    //             // double time = timer_.GetFPGATimestamp().value() - coneGrabTimerStartTime_;

    //             double intermediatePhi = 140;
    //             if (atConeIntakePos && (arm_.getState() == TwoJointArm::HOLDING_POS || arm_.getState() == TwoJointArm::STOPPED))
    //             {
    //                 arm_.setJointPath(TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], intermediatePhi);
    //                 // arm_.setPosTo(TwoJointArmProfiles::STOWED);
    //             }
    //             else if(abs(arm_.getTheta() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - intermediatePhi) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && (arm_.getState() == TwoJointArm::HOLDING_POS || arm_.getState() == TwoJointArm::STOPPED))
    //             {
    //                 arm_.setJointPath(TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3]);
    //             }
    //             else if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
    //             {
    //                 coneIntakeHalfway = false;
    //                 coneIntaking_ = false;
    //                 intakesNeededDown.getY() = false;
    //                 coneIntakeDown_ = false;
    //             }

    //             if(abs(arm_.getPhi() - intermediatePhi) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    //             {
    //                 intakesNeededDown.getY() = true;
    //             }
    //         }
    //     }
    // }
    else
    {
        coneGrabTimerStarted_ = false;
        grabbedCone_ = false;
    }
    // else if (arm_.getPosition() == TwoJointArmProfiles::CUBE_INTAKE && arm_.getState() != TwoJointArm::MANUAL)
    // {
    //     arm_.setPosTo(TwoJointArmProfiles::STOWED);
    // }

    if (controls_.getPressedOnce(TOGGLE_CLAW))
    {
        if (/*!arm_.intaking() && */ !cubeIntaking_ && !coneIntaking_)
        {
            arm_.setClaw(!arm_.getClawOpen());
        }
    }

    if (/*dPadRightPressed*/ controls_.getPOVDown(CUBE_INTAKE))
    {
        if (!arm_.isForward())
        {
            if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
            {
                arm_.toggleForward();
            }
            else
            {
                arm_.setPosTo(TwoJointArmProfiles::STOWED);
            }
        }
        else
        {
            bool atGroundPos = (abs(arm_.getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(arm_.getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
            // arm_.setPosTo(TwoJointArmProfiles::GROUND);
            if (arm_.getPosition() == TwoJointArmProfiles::STOWED && arm_.getState() == TwoJointArm::HOLDING_POS)
            {
                arm_.setJointPath(0, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]);
            }
            else if (arm_.getPosition() != TwoJointArmProfiles::STOWED && !atGroundPos && (!arm_.isArmMoving()))
            {
                arm_.setPosTo(TwoJointArmProfiles::STOWED);
            }
        }
        // if(!coneIntaking_ && !cubeIntaking_)
        // {
        //     coneIntakeDown_ = !coneIntakeDown_;
        // }
    }
    
    if (controls_.getPressedOnce(INTAKE))
    {
        if (/*!arm_.intaking() && */ !cubeIntaking_ && !coneIntaking_)
        {
            if (arm_.getClawWheelSpeed() == ClawConstants::INTAKING_SPEED)
            {
                arm_.setClawWheels(0);
            }
            else
            {
                arm_.setClawWheels(ClawConstants::INTAKING_SPEED);
            }
        }
    }
    else if (controls_.getPressedOnce(OUTAKE))
    {
        if (/*!arm_.intaking() && */ !cubeIntaking_ && !coneIntaking_)
        {
            if (arm_.getClawWheelSpeed() == ClawConstants::OUTAKING_SPEED)
            {
                arm_.setClawWheels(0);
            }
            else
            {
                arm_.setClawWheels(ClawConstants::OUTAKING_SPEED);
            }
        }
    }

    bool cutoutIntakePressed = controls_.getPOVDown(CUTOUT_INTAKE);
    bool cutoutOutakePressed = controls_.getPOVDownOnce(CUTOUT_OUTAKE);
    if(cutoutIntakePressed)
    {
        cubeGrabber_.Intake();
    }
    else if(cutoutOutakePressed)
    {
        if(cubeGrabber_.getState() == CubeGrabber::OUTTAKING)
        {
            cubeGrabber_.OuttakeSlow();
        }
        else
        {
            cubeGrabber_.Outtake();
        }
    }

    if (cubeIntakeNeededDown)
    {
        cubeIntake_.Deploy();
        if (cubeIntaking_)
        {
            cubeIntake_.setRollerMode(PneumaticsIntake::INTAKE);
        }
        else
        {
            // cubeIntake_.setRollerMode(PneumaticsIntake::INTAKE);
            cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
        }
    }
    else
    {
        cubeIntake_.Stow();
        cubeIntake_.setRollerMode(PneumaticsIntake::STOP);
    }

    frc::SmartDashboard::PutBoolean("Cube Intake Down", cubeIntakeNeededDown);
    // frc::SmartDashboard::PutBoolean("Cone Intake Down", intakesNeededDown.getY() || coneIntakeDown_);
    // frc::SmartDashboard::PutBoolean("Cone Intake Halfway", coneIntakeHalfway);
}

void Robot::DisabledInit()
{
    arm_.stop();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
    PCM.Disable();
    // pneumaticHub_.DisableCompressor();
}

void Robot::DisabledPeriodic()
{
    frc::SmartDashboard::PutNumber("Theta", arm_.getTheta());
    swerveDrive_.reset();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
    arm_.checkPos();
    cubeGrabber_.Stop();

    // frc::SmartDashboard::PutBoolean("XDown", controls_.lineupTrimXDownPressed());
    // frc::SmartDashboard::PutBoolean("XUp", controls_.lineupTrimXUpPressed());
    // frc::SmartDashboard::PutBoolean("YDown", controls_.lineupTrimYDownPressed());
    // frc::SmartDashboard::PutBoolean("YUp", controls_.lineupTrimYUpPressed());

    // frc::SmartDashboard::PutNumber("PS", controls_.checkPSButtons());

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
