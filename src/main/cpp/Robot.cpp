// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() : autoPaths_(swerveDrive_, arm_)
{

    AddPeriodic(
        [&]
        {
            double yaw = navx_->GetYaw() - yawOffset_;
            Helpers::normalizeAngle(yaw);
            frc::SmartDashboard::PutNumber("yaw", yaw);
            swerveDrive_->periodic(yaw);
            arm_->periodic();

            if (frc::DriverStation::IsAutonomous() && frc::DriverStation::IsEnabled())
            {
                autoPaths_.periodic();
            }
            else
            {
                bool armMoving = (arm_->getState() != TwoJointArm::STOPPED && arm_->getState() != TwoJointArm::HOLDING_POS);
                bool armOut = (arm_->getPosition() != TwoJointArmProfiles::STOWED && arm_->getPosition() != TwoJointArmProfiles::CONE_INTAKE && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE);
                swerveDrive_->teleopPeriodic(controls_, arm_->isForward(), (armMoving || armOut));
            }

            // if (frc::DriverStation::IsEnabled())
            // {
            //     arm_->periodic();
            //     bool armMoving = (arm_->getState() != TwoJointArm::STOPPED && arm_->getState() != TwoJointArm::HOLDING_POS);
            //     swerveDrive_->periodic(yaw, controls_, arm_->isForward(), armMoving);
            // }
        },
        5_ms, 2_ms);
}

void Robot::RobotInit()
{
    auto1Chooser_.SetDefaultOption("Preloaded Cone", AutoPaths::PRELOADED_CONE);
    auto1Chooser_.AddOption("Preloaded Cube", AutoPaths::PRELOADED_CUBE);
    // auto1Chooser_.AddOption("First Cone", AutoPaths::FIRST_CONE);
    // auto1Chooser_.AddOption("First Cube", AutoPaths::FIRST_CUBE);
    // auto1Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    // auto1Chooser_.AddOption("Second Cube", AutoPaths::SECOND_CUBE);
    auto1Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto1Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto1Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto1Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    frc::SmartDashboard::PutData("First Auto Stage", &auto1Chooser_);

    auto2Chooser_.SetDefaultOption("First Cube", AutoPaths::FIRST_CUBE);
    // auto2Chooser_.AddOption("Preloaded Cone", AutoPaths::PRELOADED_CONE);
    // auto2Chooser_.AddOption("Preloaded Cube", AutoPaths::PRELOADED_CUBE);
    // auto2Chooser_.AddOption("First Cone", AutoPaths::FIRST_CONE); //this
    // auto2Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    // auto2Chooser_.AddOption("Second Cube", AutoPaths::SECOND_CUBE);
    auto2Chooser_.AddOption("Auto Dock", AutoPaths::AUTO_DOCK);
    auto2Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto2Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto2Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    frc::SmartDashboard::PutData("Second Auto Stage", &auto2Chooser_);

    auto3Chooser_.SetDefaultOption("AUTO_DOCK", AutoPaths::AUTO_DOCK);
    // auto3Chooser_.AddOption("Preloaded Cone", AutoPaths::PRELOADED_CONE);
    // auto3Chooser_.AddOption("Preloaded Cube", AutoPaths::PRELOADED_CUBE);
    auto3Chooser_.AddOption("First Cone", AutoPaths::FIRST_CONE);
    auto3Chooser_.AddOption("First Cube", AutoPaths::FIRST_CUBE);
    // auto3Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE); //this
    auto3Chooser_.AddOption("Second Cube", AutoPaths::SECOND_CUBE);
    auto3Chooser_.AddOption("Nothing", AutoPaths::NOTHING);
    auto3Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto3Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    frc::SmartDashboard::PutData("Third Auto Stage", &auto3Chooser_);

    auto4Chooser_.SetDefaultOption("Nothing", AutoPaths::NOTHING);
    // auto4Chooser_.AddOption("Preloaded Cone", AutoPaths::PRELOADED_CONE);
    // auto4Chooser_.AddOption("Preloaded Cube", AutoPaths::PRELOADED_CUBE);
    // auto4Chooser_.AddOption("First Cone", AutoPaths::FIRST_CONE);
    // auto4Chooser_.AddOption("First Cube", AutoPaths::FIRST_CUBE);
    auto4Chooser_.AddOption("Second Cone", AutoPaths::SECOND_CONE);
    auto4Chooser_.AddOption("Second Cube", AutoPaths::SECOND_CUBE);
    auto4Chooser_.AddOption("AUTO_DOCK", AutoPaths::AUTO_DOCK);
    auto4Chooser_.AddOption("Drive Back Dumb", AutoPaths::DRIVE_BACK_DUMB);
    auto4Chooser_.AddOption("Wait Five Seconds", AutoPaths::WAIT_5_SECONDS);
    frc::SmartDashboard::PutData("Fourth Auto Stage", &auto4Chooser_);

    sideChooser_.SetDefaultOption("Right", false);
    sideChooser_.AddOption("Left", true);
    frc::SmartDashboard::PutData("Auto Side", &sideChooser_);

    cubeIntaking_ = false;
    coneIntaking_ = false;
    frc::SmartDashboard::PutNumber("Test Volts", 3);

    try
    {
        navx_ = new AHRS(frc::SerialPort::kUSB);
    }
    catch (const exception &e)
    {
        cout << e.what() << endl;
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
    frc::SmartDashboard::PutBoolean("Shoulder Brake", arm_->shoulderBrakeEngaged());
    frc::SmartDashboard::PutBoolean("Elbow Brake", arm_->elbowBrakeEngaged());
    frc::SmartDashboard::PutBoolean("Forward", arm_->isForward());
    frc::SmartDashboard::PutBoolean("Pos Known", !arm_->posUnknown());
    frc::SmartDashboard::PutBoolean("Intaking Cone", coneIntaking_);
    frc::SmartDashboard::PutBoolean("Intaking Cube", cubeIntaking_);
    frc::SmartDashboard::PutBoolean("ESTOPPED", arm_->isEStopped());

    frc::SmartDashboard::PutString("Arm State", arm_->getStateString());
    frc::SmartDashboard::PutString("Arm Pos", arm_->getPosString());
    frc::SmartDashboard::PutString("Arm Set Pos", arm_->getSetPosString());

    frc::SmartDashboard::PutNumber("x", swerveDrive_->getX());
    frc::SmartDashboard::PutNumber("y", swerveDrive_->getY());
    frc::SmartDashboard::PutNumber("Theta", arm_->getTheta());
    frc::SmartDashboard::PutNumber("Phi", arm_->getPhi());
    frc::SmartDashboard::PutNumber("Theta vel", arm_->getThetaVel());
    frc::SmartDashboard::PutNumber("Phi vel", arm_->getPhiVel());
    // frc::SmartDashboard::PutNumber("Theta Volts", arm_->getThetaVolts());
    // frc::SmartDashboard::PutNumber("Phi Volts", arm_->getPhiVolts());
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
    arm_->stop();
    arm_->resetIntaking();
    arm_->zeroArmsToStow();

    AutoPaths::Path action1 = auto1Chooser_.GetSelected();
    AutoPaths::Path action2 = auto2Chooser_.GetSelected();
    AutoPaths::Path action3 = auto3Chooser_.GetSelected();
    AutoPaths::Path action4 = auto4Chooser_.GetSelected();
    autoPaths_.setMirrored(sideChooser_.GetSelected());

    // m_autoSelected = frc::SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
    // fmt::print("Auto selected: {}\n", m_autoSelected);
    autoPaths_.setActions(action1, action2, action3, action4);

    swerveDrive_->reset();

    navx_->ZeroYaw();
    yawOffset_ = autoPaths_.initYaw();
    swerveDrive_->setPos(autoPaths_.initPos());

    autoPaths_.startTimer();
}

void Robot::AutonomousPeriodic()
{
    bool clawOpen = autoPaths_.getClawOpen();
    double wheelSpeed = autoPaths_.getWheelSpeed();
    TwoJointArmProfiles::Positions armPosition = autoPaths_.getArmPosition();
    bool forward = autoPaths_.getForward();

    pair<bool, bool> intakesNeededDown = arm_->intakesNeededDown();
    cubeIntaking_ = autoPaths_.cubeIntaking();
    coneIntaking_ = autoPaths_.coneIntaking();

    //frc::SmartDashboard::PutBoolean("CUBE INTAKING", cubeIntaking_);
    if (cubeIntaking_)
    {
        if (arm_->isForward())
        {
            if (arm_->getPosition() != TwoJointArmProfiles::STOWED)
            {
                armPosition = TwoJointArmProfiles::STOWED;
            }
            else
            {
                arm_->toggleForwardCubeIntake();
            }
        }
        else
        {
            if (arm_->getPosition() != TwoJointArmProfiles::STOWED && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
            {
                armPosition = TwoJointArmProfiles::STOWED;
            }
            else
            {
                intakesNeededDown.first = true;
                armPosition = TwoJointArmProfiles::CUBE_INTAKE;
                wheelSpeed = ClawConstants::INTAKING_SPEED;
                clawOpen = true;
            }
        }
    }
    else if (forward && arm_->getPosition() == TwoJointArmProfiles::CUBE_INTAKE)
    {
        arm_->toggleForwardCubeIntake();
    }
    else
    {
        if (arm_->getPosition() == TwoJointArmProfiles::CUBE_INTAKE)
        {
            armPosition = TwoJointArmProfiles::STOWED;
        }
    }

    if (arm_->isForward() != forward)
    {
        arm_->toggleForward();
    }

    arm_->setPosTo(armPosition);
    arm_->setClawWheels(wheelSpeed);
    arm_->setClaw(clawOpen);

    if (intakesNeededDown.first)
    {
        // TODO put down cube intake
    }

    if (intakesNeededDown.second)
    {
        // TODO put down cone intake
    }
}

void Robot::TeleopInit()
{
    // frc::SmartDashboard::PutNumber("Set Theta", 0);
    // frc::SmartDashboard::PutNumber("Set Phi", 0);
    // frc::SmartDashboard::PutNumber("Swerve Volts", 0);

    cubeIntaking_ = false;
    coneIntaking_ = false;
    PCM.EnableDigital();

    arm_->stop();
    arm_->resetIntaking();
}

void Robot::TeleopPeriodic()
{
    controls_->periodic();
    swerveDrive_->setScoringPos(controls_->checkScoringButtons());

    pair<bool, bool> intakesNeededDown = arm_->intakesNeededDown();

    if (controls_->fieldOrient())
    {
        navx_->ZeroYaw();
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
        {
            yawOffset_ = 90;
        }
        else
        {
            yawOffset_ = -90;
        }
    }

    if (controls_->dPadDownPressed())
    {
        arm_->zeroArms();
    }

    if (controls_->lXTriggerPressed())
    {
        arm_->manualControl(controls_->xboxLJoyY() * TwoJointArmConstants::SHOULDER_ARM_MAX_VEL * 0.5, controls_->xboxRJoyY() * TwoJointArmConstants::ELBOW_ARM_MAX_VEL * 0.5);
    }
    else
    {
        // if(controls_->aPressed())
        // {
        //     arm_->goToPos(frc::SmartDashboard::GetNumber("Set Theta", 0), frc::SmartDashboard::GetNumber("Set Phi", 0));
        // }
        // else
        // {
        //     //arm_->stop();
        // }

        if (arm_->getState() == TwoJointArm::MANUAL)
        {
            arm_->stop();
        }

        if (controls_->xPressed())
        {
            if (!coneIntaking_ && !cubeIntaking_)
            {
                arm_->setPosTo(TwoJointArmProfiles::PLAYER_STATION);
            }
        }
        else if (controls_->bPressed())
        {
            if (!coneIntaking_ && !cubeIntaking_)
            {
                if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue && swerveDrive_->getX() > FieldConstants::FIELD_LENGTH / 2)
                {
                    arm_->setPosTo(TwoJointArmProfiles::MID);
                }
                else if (swerveDrive_->getX() < FieldConstants::FIELD_LENGTH / 2)
                {
                    arm_->setPosTo(TwoJointArmProfiles::MID);
                }
                else
                {
                    int scoringPos = swerveDrive_->getScoringPos();
                    if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8)
                    {
                        arm_->setPosTo(TwoJointArmProfiles::CUBE_MID);
                    }
                    else
                    {
                        arm_->setPosTo(TwoJointArmProfiles::MID);
                    }
                }
            }
        }
        else if (controls_->aPressed())
        {
            if (!coneIntaking_ && !cubeIntaking_)
            {
                arm_->setPosTo(TwoJointArmProfiles::STOWED);
            }
        }
        else if (controls_->yPressed())
        {
            if (!coneIntaking_ && !cubeIntaking_)
            {
                int scoringPos = swerveDrive_->getScoringPos();
                if (scoringPos == 2 || scoringPos == 5 || scoringPos == 8)
                {
                    arm_->setPosTo(TwoJointArmProfiles::CUBE_HIGH);
                }
                else
                {
                    arm_->setPosTo(TwoJointArmProfiles::HIGH);
                }
            }
        }
        else if (controls_->rBumperPressed())
        {
            if (arm_->isForward() && arm_->getPosition() == TwoJointArmProfiles::STOWED)
            {
                arm_->setClaw(true);
                arm_->setPosTo(TwoJointArmProfiles::CONE_INTAKE);
            }
        }
        else if (controls_->dPadLeftPressed())
        {
            if (arm_->getState() == TwoJointArm::HOLDING_POS)
            {
                if (cubeIntaking_)
                {
                    arm_->setPosTo(TwoJointArmProfiles::STOWED);
                    //arm_->setClawWheels(0);
                    // arm_->setClaw(false);
                }
                cubeIntaking_ = !cubeIntaking_;
                coneIntaking_ = false;
            }
        }
        else if (controls_->dPadRightPressed())
        {
            // if (arm_->getState() == TwoJointArm::HOLDING_POS)
            // {
            //     if (coneIntaking_)
            //     {
            //         arm_->setPosTo(TwoJointArmProfiles::STOWED);
            //         arm_->setClawWheels(0);
            //         // arm_->setClaw(false);
            //     }
            //     coneIntaking_ = !coneIntaking_;
            //     cubeIntaking_ = false;
            // }

        }
        else if (controls_->dPadUpPressed())
        {
            if (cubeIntaking_)
            {
                // arm_->toggleForwardCubeIntake();
                // cubeIntaking_ = false;
            }
            else
            {
                arm_->toggleForward();
            }
        }
        else
        {
            arm_->setEStopped(false);
        }
    }

    if (controls_->lBumperPressed())
    {
        arm_->stop();
        arm_->resetIntaking();
        coneIntaking_ = false;
        cubeIntaking_ = false;
        arm_->setEStopped(true);
    }

    if (cubeIntaking_)
    {
        if (arm_->isForward())
        {
            if (arm_->getPosition() != TwoJointArmProfiles::STOWED)
            {
                arm_->setPosTo(TwoJointArmProfiles::STOWED);
            }
            else
            {
                arm_->toggleForwardCubeIntake();
            }
        }
        else
        {
            // if (arm_->getPosition() != TwoJointArmProfiles::STOWED && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
            // {
            //     arm_->setPosTo(TwoJointArmProfiles::STOWED);
            // }
            // else
            // {
            intakesNeededDown.first = true;
            arm_->setPosTo(TwoJointArmProfiles::CUBE_INTAKE);
            // arm_->setClawWheels(ClawConstants::INTAKING_SPEED);
            // arm_->setClaw(true);
            // }
        }

        arm_->setClawWheels(ClawConstants::INTAKING_SPEED);
        arm_->setClaw(true);
    }
    else if (arm_->getPosition() == TwoJointArmProfiles::CUBE_INTAKE)
    {
        arm_->setPosTo(TwoJointArmProfiles::STOWED);
    }

    if (coneIntaking_)
    {
    }
    else
    {
    }

    if (controls_->rJoyTriggerPressed())
    {
        if (!arm_->intaking() && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
        {
            arm_->setClaw(!arm_->getClawOpen());
        }
    }

    if (controls_->intakePressed())
    {
        if (!arm_->intaking() && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
        {
            if (arm_->getClawWheelSpeed() == ClawConstants::INTAKING_SPEED)
            {
                arm_->setClawWheels(0);
            }
            else
            {
                arm_->setClawWheels(ClawConstants::INTAKING_SPEED);
            }
        }
    }
    else if (controls_->outakePressed())
    {
        if (!arm_->intaking() && arm_->getPosition() != TwoJointArmProfiles::CUBE_INTAKE)
        {
            if (arm_->getClawWheelSpeed() == ClawConstants::OUTAKING_SPEED)
            {
                arm_->setClawWheels(0);
            }
            else
            {
                arm_->setClawWheels(ClawConstants::OUTAKING_SPEED);
            }
        }
    }

    // if(controls_->lLowerButtonPressed())
    // {

    // }
    // else if(controls_->rLowerButtonPressed())
    // {

    // }

    if (intakesNeededDown.first)
    {
        // TODO put down cube intake
    }

    if (intakesNeededDown.second)
    {
        // TODO put down cone intake
    }

    frc::SmartDashboard::PutBoolean("Cube Intake Down", intakesNeededDown.first);
    frc::SmartDashboard::PutBoolean("Cone Intake Down", intakesNeededDown.second);
}

void Robot::DisabledInit()
{
    arm_->stop();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
    PCM.Disable();
    // pneumaticHub_.DisableCompressor();
}

void Robot::DisabledPeriodic()
{
    frc::SmartDashboard::PutNumber("Theta", arm_->getTheta());
    swerveDrive_->reset();
    autoPaths_.setActionsSet(false);
    autoPaths_.setPathSet(false);
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
