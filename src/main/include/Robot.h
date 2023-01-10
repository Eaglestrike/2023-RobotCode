#pragma once

#include <string>
#include <AHRS.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "SwerveDrive.h"
#include "Limelight.h"
#include "frc/Joystick.h"
#include "Constants.h"

class Robot : public frc::TimedRobot {
 public:
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
    void SimulationInit() override;
    void SimulationPeriodic() override;

 private:
   AHRS * navx_; //can't be initialized by compiler because doesn't have a constructor :(
   SwerveDrive * swerveDrive_; //pointer because it relies on navx being initialized
   Limelight limelight_;

   frc::Joystick ljoy{InputConstants::LJOY_PORT};
   frc::Joystick rjoy{InputConstants::RJOY_PORT};

   double joy_val_to_mps(double val);
   double joy_rot_to_rps(double rot); 
};
