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
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  controls::TorqueCurrentFOC motorRequest{0_A};

  double torque = frc::SmartDashboard::GetNumber("T", 0.0);; // TODO
  double current = torque / MotorConstants::Kt;

  std::cout << "torque: " << torque << " current: " << current << std::endl;

  m_talon.SetControl(motorRequest.WithOutput(units::current::ampere_t(current)));
  auto currentVelocity = m_talon.GetRotorVelocity().GetValue();
  auto acceleration = ((currentVelocity - lastVelocity) * 2 * M_PI / 0.02).value();
  auto armAngle = m_talon.GetRotorPosition().GetValue();
  auto angle = armAngle.value() * 2 * M_PI;
  auto torque_real = ArmConstants::I * acceleration + ArmConstants::mass * 9.81 * ArmConstants::length / 2 * std::cos(angle);

  std::cout << "torque_real: " << torque_real << std::endl;
  frc::SmartDashboard::PutNumber("torque_real", torque_real);
  frc::SmartDashboard::PutNumber("ange", angle);
  frc::SmartDashboard::PutNumber("acceleration", acceleration);

  lastVelocity = currentVelocity;
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
