#include "Robot.h"

#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  navx_ = new AHRS(frc::SPI::Port::kMXP);
  swerveDrive_ = new SwerveDrive(navx_, limelight_);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  swerveDrive_->initializeAutoTraj(SwerveConstants::twoBallPath); //todo would be done with auto chooser
}

void Robot::AutonomousPeriodic() {
  // swerveDrive_->setState(SwerveDrive::State::PATH_FOLLOW); //todo would be moved into auto executor
  // swerveDrive_->Periodic(0); //dummy turret value of 0 for testing purposes
}

void Robot::TeleopInit() {
  navx_->ZeroYaw();
  swerveDrive_->setState(SwerveDrive::State::DRIVE);
}

void Robot::TeleopPeriodic() {

  double dx = ljoy.GetRawAxis(1);
  double dy = ljoy.GetRawAxis(0);
  double dtheta = rjoy.GetX();
  dx = abs(dx) < 0.1 ? 0.0: dx; 
  dy = abs(dy) < 0.1 ? 0.0: dy;
  dtheta = abs(dtheta) < 0.05 ? 0.0: dtheta;

  joy_val_to_mps(dx);
  joy_val_to_mps(dy);
  joy_rot_to_rps(dtheta);

  swerveDrive_->Periodic(
    units::meters_per_second_t{dx},
    units::meters_per_second_t{dy},
    units::radians_per_second_t{0.7*dtheta},
    0);

  //REMEMBER TO COMMENT IN USE OF SPEED PID BEFORE TESTING
 // swerveDrive_->Periodic(0, 0_mps); //go 1 meter per second in the x direction. for testing speed tuning
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
