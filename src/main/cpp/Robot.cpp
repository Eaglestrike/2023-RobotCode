// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Constants.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

using namespace ctre::phoenixpro;

void Robot::RobotInit() {
  frc::SmartDashboard::PutNumber("T", 0.0);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  frc::SmartDashboard::PutNumber("T", 0.0);
  lastTime = frc::Timer::GetFPGATimestamp();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  controls::TorqueCurrentFOC motorRequest{0_A};

  double torque = frc::SmartDashboard::GetNumber("T", 0.0); // TODO

  double current = torque / MotorConstants::Kt;

  m_talon.SetControl(motorRequest.WithOutput(units::current::ampere_t(current)));

  frc::SmartDashboard::PutNumber("current", current);
  frc::SmartDashboard::PutNumber("Supply current", m_talon.GetSupplyCurrent().GetValue().value());

  auto currentVelocity = m_talon.GetRotorVelocity().GetValue();

  //for acceleration-based torque comparison
  if (currentVelocity != lastVelocity) {
    double deltaT = frc::Timer::GetFPGATimestamp().value() - lastTime.value();
    auto acceleration = ((currentVelocity.value() - lastVelocity.value()) * 2.0 * M_PI / deltaT);
    auto armAngle = m_talon.GetRotorPosition().GetValue();
    auto angle = armAngle.value() * 2 * M_PI;
    //no gravity if arm is balanced
    auto torque_real = ArmConstants::I * acceleration; // + ArmConstants::mass * 9.81 * ArmConstants::length / 4 * std::cos(angle);

    frc::SmartDashboard::PutNumber("currentVelocity", currentVelocity.value() * 2 * M_PI);
    frc::SmartDashboard::PutNumber("lastVelocity", lastVelocity.value() * 2 * M_PI);
    frc::SmartDashboard::PutNumber("delta T", deltaT);
    frc::SmartDashboard::PutNumber("torque_real", torque_real);
    frc::SmartDashboard::PutNumber("angle", angle);
    frc::SmartDashboard::PutNumber("acceleration", acceleration);
    
    lastVelocity = units::angular_velocity::turns_per_second_t{currentVelocity.value()};
    lastTime = frc::Timer::GetFPGATimestamp();
  }

  
  
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
  m_talon.SetRotorPosition(0_deg);
}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
