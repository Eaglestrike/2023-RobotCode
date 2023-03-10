// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() : autoPaths_(swerveDrive_, arm_)
{}

void Robot::RobotInit()
{
    coneIntake_.RobotInit();
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
}

void Robot::AutonomousPeriodic()
{
}

void Robot::TeleopInit()
{
}

void Robot::TeleopPeriodic()
{
    if (controls_->xDown()) {
        if (controls_->lXTriggerDown()) {
            coneIntake_.WaitForCone();
        } else {
            coneIntake_.Ground();
        }
    } else if (controls_->yDown()) {
        coneIntake_.Middle();
    } else {
        coneIntake_.Stow();
    }
    coneIntake_.PutDebug();

    coneIntake_.Periodic();

    frc::SmartDashboard::PutBoolean("Arm good", coneIntake_.IsClearForArm());
}

void Robot::DisabledInit()
{
    coneIntake_.DisabledInit();
}

void Robot::DisabledPeriodic()
{
    coneIntake_.PutDebug();
    if (controls_->aDown()) {
        coneIntake_.Reset();
        coneIntake_.SetConstants();
    }
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
