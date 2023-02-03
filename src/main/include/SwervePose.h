#pragma once

#include <math.h>
#include "Constants.h"

class SwervePose
{
    public:
        SwervePose(double x, double y, double yaw, double xVel, double yVel, double yawVel, double xAcc, double yAcc, double yawAcc);
        SwervePose(double x, double y, double yaw, double yawDist);
        double getX();
        double getY();
        double getYaw();
        double getYawDist();
        double getXVel();
        double getYVel();
        double getYawVel();
        double getXAcc();
        double getYAcc();
        double getYawAcc();

        double distTo(SwervePose endPose);
        double angTo(SwervePose endPose);

    private:
        double x_, y_, yaw_, yawDist_, xVel_, yVel_, yawVel_, xAcc_, yAcc_, yawAcc_;
};