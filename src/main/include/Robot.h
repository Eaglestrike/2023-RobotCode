#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "Arm.h"
#include "Constants.h"
#include "Controls.h"

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
   Arm arm;
   Controls controller;
   double moveMetersPerSecond = 0.1;
   double moveRadiansPerSecond = 0.3;
   double maxAmpsBase = 5.0;
   double maxAmpsTop = 4.0;
};
