#include "Robot.h"

#include <fmt/core.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>

//Initialize pointer objects
void Robot::RobotInit() {
  arm.init();
  //frc::SmartDashboard::PutNumber("Switch to one to set new target", 0);
  //hi this is caleb
  //caleb says hi
  //this is caleb when the car comes by
  //husrfhvgaehgvadhgvadhga
}

void Robot::RobotPeriodic() {
    double xStrafe = moveMetersPerSecond*controller.getXStrafe()*0.02;
    double yStrafe = moveMetersPerSecond*controller.getYStrafe()*0.02;
    if((xStrafe != 0) || (yStrafe != 0)){
        arm.moveTarget(xStrafe, yStrafe);
    }
    if (controller.A_IsPressed()) {
        arm.resetTarget();
    }
    arm.Periodic();
}

void Robot::AutonomousInit() {
}

//commented out for testing purposes
void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  arm.firstRun = true;
}

void Robot::TeleopPeriodic() {
  /*
  int newTarget = frc::SmartDashboard::GetNumber("Switch to one to set new target", 0);
  if(newTarget == 1){
    double targetX = frc::SmartDashboard::GetNumber("Target X", 0.0);
    double targetZ = frc::SmartDashboard::GetNumber("Target Z", 0.0);
    arm.setTarget(targetX, targetZ);
    frc::SmartDashboard::PutNumber("Switch to one to set new target", 0);
  }
  */
  if (controller.X_IsPressed()) {
    arm.setTarget(-1.27, 0.995);
  }
  if (controller.Y_IsPressed()) {
    arm.setTarget(-1.27, 1.1);
  }
  if (controller.B_IsPressed()) {
    arm.setTarget(-0.2, 1.1);
  }
  arm.TeleopPeriodic();
  
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
  /*
  int newTarget = frc::SmartDashboard::GetNumber("Switch to one to set new target", 0);
  if(newTarget == 1){
    double targetX = frc::SmartDashboard::GetNumber("Target X", 0.0);
    double targetZ = frc::SmartDashboard::GetNumber("Target Z", 0.0);
    arm.setTarget(targetX, targetZ);
    frc::SmartDashboard::PutNumber("Switch to one to set new target", 0);
  }
  */
  arm.DisabledPeriodic();
}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  arm.TestPeriodic(maxAmpsBase*controller.getX2Strafe(),
                  maxAmpsTop*controller.getY2Strafe()
                  );
  if (controller.A_IsPressed()) {
    arm.resetEncoder();
    arm.resetOffsets();
  }
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
