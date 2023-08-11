#pragma once

#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Helpers/Helpers.h"
#include "SwervePose.h"
#include "SwerveConstants.h"

class SwerveTrajectory
{
    public:
        SwerveTrajectory(Poses::SwervePose startPose, Poses::SwervePose endPose, double yawAccelTime, double yawCruiseTime, double yawCruiseDist, double yawCruiseVel, 
        double linYawAccelTime, double linYawCruiseTime, double linYawCruiseDist, double linYawCruiseVel, 
        double linYawDeccelTime, double actualYawDist, double endVel, 
        double linAccelTime, double linCruiseTime, double linDeccelTime, double linCruiseVel, double linCruiseDist, int yawDirection, 
        double maxLA, double maxLV, double maxAA, double maxAV);

        Poses::SwervePose getPose(double time);
        double getTotalTime();
    private:
        Poses::SwervePose startPose_, endPose_;
        double yawAccelTime_, yawCruiseTime_, yawCruiseDist_, yawCruiseVel_, 
        linYawAccelTime_, linYawCruiseTime_, linYawCruiseDist_, linYawCruiseVel_, 
        linYawDeccelTime_, actualYawDist_, endVel_, 
        linAccelTime_, linCruiseTime_, linDeccelTime_, linCruiseVel_, linCruiseDist_;
        int yawDirection_;
        double MAX_LA, MAX_LV, MAX_AA, MAX_AV;
        
};