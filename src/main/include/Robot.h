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
#include <frc/Compressor.h>
#include <frc/PneumaticHub.h>
#include <frc/Timer.h>

#include "AHRS.h"

#include "GeneralConstants.h"

#include "Controller/Controller.h"

#include "AutoPaths.h"
#include "Drivebase/SwerveDrive.h"

#include "Intake/PneumaticsIntake.h"
#include "Intake/CubeGrabber.h"
#include "Vision/SocketClient.h"

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
    frc::SendableChooser<AutoPaths::Path> auto4Chooser_;
    frc::SendableChooser<bool> sideChooser_;

    AHRS *navx_;
    frc::Compressor PCM{0, frc::PneumaticsModuleType::CTREPCM};

    Controller controls_;
    SwerveDrive swerveDrive_;
    TwoJointArm arm_;
    AutoPaths autoPaths_{swerveDrive_, arm_};
    PneumaticsIntake cubeIntake_{false, false};
    CubeGrabber cubeGrabber_;
    SocketClient socketClient_;

    double yawOffset_;

    frc::Timer timer_;
    double coneGrabTimerStartTime_;
    bool coneGrabTimerStarted_;

    bool cubeIntaking_, coneIntaking_, coneIntakeDown_, armsZeroed_, grabbedCone_;
    int scoringLevel_/*, psType_*/;

};
