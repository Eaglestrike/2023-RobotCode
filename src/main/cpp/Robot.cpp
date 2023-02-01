#include "Robot.h"

#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>

//Initialize pointer objects
void Robot::RobotInit() {
  navx_ = new AHRS(frc::SPI::Port::kMXP);
  swerveDrive_ = new SwerveDrive(navx_);

  // logging setup
  frc::DataLogManager::Start();

  wpi::log::DataLog& log = frc::DataLogManager::GetLog();
  swerveX = wpi::log::DoubleLogEntry(log, "swerve.X");
  swerveY = wpi::log::DoubleLogEntry(log, "swerve.Y");
}

void Robot::RobotPeriodic() {
  
  // log value of registered data fields
  swerveX.Append(swerveDrive_->getX());
  swerveY.Append(swerveDrive_->getY());

  frc::SmartDashboard::PutNumber("Tag x", 0);
  frc::SmartDashboard::PutNumber("Tag y", 0);
  frc::SmartDashboard::PutNumber("Tag z", 0);

  frc::SmartDashboard::PutNumber("Tag roll (x)", 0);
  frc::SmartDashboard::PutNumber("Tag pitch (y)", 0);
  frc::SmartDashboard::PutNumber("Tag yaw (z)", 0);

  frc::SmartDashboard::PutNumber("Tag num", 0);
  
}


void Robot::AutonomousInit() {
  swerveDrive_->initializeAutoTraj(SwerveConstants::testPath); //todo would be done with auto chooser depending on auto mode
}

//commented out for testing purposes
void Robot::AutonomousPeriodic() {
  swerveDrive_->setState(SwerveDrive::State::PATH_FOLLOW); //todo would be moved into auto executor
  swerveDrive_->Periodic( 0_mps, 0_mps, 0_rad / 1_s);
}

void Robot::TeleopInit() {
  navx_->ZeroYaw();
  swerveDrive_->setState(SwerveDrive::State::DRIVE);
}

void Robot::TeleopPeriodic() {

  double x = frc::SmartDashboard::GetNumber("Tag x", 0);
  double y = frc::SmartDashboard::GetNumber("Tag y", 0);
  double z = frc::SmartDashboard::GetNumber("Tag z", 0);

  double ax = frc::SmartDashboard::GetNumber("Tag roll (x)", 0);
  double ay = frc::SmartDashboard::GetNumber("Tag pitch (y)", 0);
  double az = frc::SmartDashboard::GetNumber("Tag yaw (z)", 0);

  double num = frc::SmartDashboard::GetNumber("Tag num", 0);

  frc::Translation3d tagTrans = {units::meter_t{x}, units::meter_t{y}, units::meter_t{z}};
  frc::Rotation3d tagRot = {units::degree_t{ax}, units::degree_t{ay}, units::degree_t{az}};
  frc::Pose3d tagPose = {tagTrans, tagRot};
  
  poseEstimator_.updateField(tagPose, num);


  //MAKE SURE THIS IS THE CORRECT SMARTDASHBOARD JETSON FORMAT
  std::vector<double> data = frc::SmartDashboard::GetNumberArray("data", {});
    
  frc::Translation3d trans{units::meter_t{data[0]}, units::meter_t{data[1]}, units::meter_t{data[2]}};
  frc::Rotation3d rot{units::degree_t{data[3]}, units::degree_t{data[4]}, units::degree_t{data[5]}};
  frc::Pose3d pose{trans, rot};

  frc::SmartDashboard::PutNumber("Robot x", poseEstimator_.getPose(pose, data[6]).X().value());
  frc::SmartDashboard::PutNumber("Robot y", poseEstimator_.getPose(pose, data[6]).Y().value());


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
    units::radians_per_second_t{vtheta});

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
