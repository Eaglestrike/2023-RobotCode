// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <sstream>
#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/DriverStation.h>
#include "AHRS.h"
#include "Constants.h"
#include "Controls.h"
#include "SwerveDrive.h"
#include "AutoPaths.h"
#include "TwoJointArm.h"

class Robot : public frc::TimedRobot
{
public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void TestInit() override;
    void TestPeriodic() override;

private:
    frc::SendableChooser<AutoPaths::Path> auto1Chooser_;
    frc::SendableChooser<AutoPaths::Path> auto2Chooser_;
    frc::SendableChooser<AutoPaths::Path> auto3Chooser_;
    frc::SendableChooser<bool> sideChooser_;

    AHRS *navx_;

    Controls* controls_ = new Controls();
    SwerveDrive* swerveDrive_ = new SwerveDrive();
    AutoPaths autoPaths_;

    double yawOffset_;

    TwoJointArm arm_;

};
