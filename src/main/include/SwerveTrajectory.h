#pragma once

#include "SwervePose.h"
#include "Helpers.h"
#include "Constants.h"
#include <math.h>

using namespace std;

class SwerveTrajectory
{
    public:
        SwerveTrajectory(SwervePose startPose, SwervePose endPose, double yawAccelTime, double yawCruiseTime, double yawCruiseDist, double yawCruiseVel, 
        double linYawAccelTime, double linYawCruiseTime, double linYawCruiseDist, double linYawCruiseVel, 
        double linYawDeccelTime, double actualYawDist, double endVel, 
        double linAccelTime, double linCruiseTime, double linDeccelTime, double linCruiseVel, double linCruiseDist, int yawDirection, 
        double maxLA, double maxLV, double maxAA, double maxAV);

        SwervePose getPose(double time);
        double getTotalTime();
    private:
        SwervePose startPose_, endPose_;
        double yawAccelTime_, yawCruiseTime_, yawCruiseDist_, yawCruiseVel_, 
        linYawAccelTime_, linYawCruiseTime_, linYawCruiseDist_, linYawCruiseVel_, 
        linYawDeccelTime_, actualYawDist_, endVel_, 
        linAccelTime_, linCruiseTime_, linDeccelTime_, linCruiseVel_, linCruiseDist_;
        int yawDirection_;
        double MAX_LA, MAX_LV, MAX_AA, MAX_AV;
        
};