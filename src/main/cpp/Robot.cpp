#include "Robot.h"

#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

//Initialize pointer objects
void Robot::RobotInit() {
  navx_ = new AHRS(frc::SPI::Port::kMXP);
  swerveDrive_ = new SwerveDrive(navx_, limelight_);
}

void Robot::RobotPeriodic() {
  limelight_.lightOn(false);
}

void Robot::AutonomousInit() {
  swerveDrive_->initializeAutoTraj(SwerveConstants::testPath); //todo would be done with auto chooser depending on auto mode
}

//commented out for testing purposes
void Robot::AutonomousPeriodic() {
  swerveDrive_->setState(SwerveDrive::State::PATH_FOLLOW); //todo would be moved into auto executor
  swerveDrive_->Periodic( 0_mps, 0_mps, 0_rad / 1_s, 0);
}

void Robot::TeleopInit() {
  navx_->ZeroYaw();
  swerveDrive_->setState(SwerveDrive::State::DRIVE);
}

void Robot::TeleopPeriodic() {

  double vx = ljoy.GetY();
  double vy = ljoy.GetX();
  double vtheta = rjoy.GetX();
  //apply deadband
  vx = abs(vx) < 0.1 ? 0.0: vx; 
  vy = abs(vy) < 0.1 ? 0.0: vy;
  vtheta = abs(vtheta) < 0.05 ? 0.0: vtheta;

  //convert joystick input to desired velocities in meters or radians per second
  vx = joy_val_to_mps(vx);
  vy = joy_val_to_mps(vy);
  vtheta = joy_rot_to_rps(vtheta);

  //drive the robot with joystick inputs
  swerveDrive_->Periodic(
    units::meters_per_second_t{vx},
    units::meters_per_second_t{vy},
    units::radians_per_second_t{vtheta},
    0);

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

//NOTE: should these two functions go in swerve?
/**
 * @returns joystick value converted to meters per second
**/
double Robot::joy_val_to_mps(double val) {
  return val* SwerveConstants::MAX_SPEED.value();
}

/**
 * @returns joystick value converted to radians per second
**/
double Robot::joy_rot_to_rps(double rot) {
  return rot * SwerveConstants::MAX_ROT.value();
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
