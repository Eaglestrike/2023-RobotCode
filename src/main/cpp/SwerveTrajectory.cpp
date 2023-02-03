#include "SwerveTrajectory.h"

SwerveTrajectory::SwerveTrajectory(SwervePose startPose, SwervePose endPose, double yawAccelTime, double yawCruiseTime, double yawCruiseDist, double yawCruiseVel, 
        double linYawAccelTime, double linYawCruiseTime, double linYawCruiseDist, double linYawCruiseVel, 
        double linYawDeccelTime, double actualYawDist, double endVel, 
        double linAccelTime, double linCruiseTime, double linDeccelTime, 
        double linCruiseVel, double linCruiseDist, int yawDirection, double maxLA, double maxLV, double maxAA, double maxAV) 
        : startPose_(startPose), endPose_(endPose), yawAccelTime_(yawAccelTime), 
        yawCruiseTime_(yawCruiseTime), yawCruiseDist_(yawCruiseDist), yawCruiseVel_(yawCruiseVel), 
        linYawAccelTime_(linYawAccelTime), linYawCruiseTime_(linYawCruiseTime), 
        linYawCruiseDist_(linYawCruiseDist), linYawCruiseVel_(linYawCruiseVel), 
        linYawDeccelTime_(linYawDeccelTime), actualYawDist_(actualYawDist), endVel_(endVel), 
        linAccelTime_(linAccelTime), linCruiseTime_(linCruiseTime), linDeccelTime_(linDeccelTime), 
        linCruiseVel_(linCruiseVel), linCruiseDist_(linCruiseDist), yawDirection_(yawDirection), MAX_LA(maxLA), MAX_LV(maxLV),
        MAX_AA(maxAA), MAX_AV(maxAV) {}
//If you're looking at this constructor... don't worry about it

SwervePose SwerveTrajectory::getPose(double time)
{
    double x, y, yaw, xVel, yVel, yawVel, xAcc, yAcc, yawAcc;
    //double dist = startPose_.distTo(endPose_);
    double ang = startPose_.angTo(endPose_);
    double yawTime = yawAccelTime_ * 2 + yawCruiseTime_;

    double linAcc, linVel, linDist;
    if(time < yawTime)
    {
        bool onlyYaw = (linYawAccelTime_ + linYawCruiseTime_ + linYawDeccelTime_ == 0);
        if(time < yawAccelTime_)
        {
            yawAcc = (onlyYaw) ? MAX_AA  * yawDirection_: MAX_AA * 0.5 * yawDirection_;
            yawVel = time * yawAcc;
            yaw = startPose_.getYaw() + yawVel * 0.5 * time;
            Helpers::normalizeAngle(yaw);
        }
        else if(time < yawAccelTime_ + yawCruiseTime_)
        {
            double prevYawAcc = (onlyYaw) ? MAX_AA  * yawDirection_: MAX_AA * 0.5 * yawDirection_;
            yawAcc = 0;
            yawVel = yawCruiseVel_;
            yaw = startPose_.getYaw() + yawAccelTime_ * yawAccelTime_ * 0.5 * prevYawAcc + yawVel * (time - yawAccelTime_);
            Helpers::normalizeAngle(yaw);
        }
        else
        {
            yawAcc = (onlyYaw) ? -MAX_AA * yawDirection_ : -MAX_AA * 0.5 * yawDirection_;
            yawVel = yawCruiseVel_ + (time - yawAccelTime_ - yawCruiseTime_) * yawAcc;
            yaw = startPose_.getYaw() + yawAccelTime_ * yawAccelTime_ * 0.5 * -yawAcc + yawCruiseVel_ * yawCruiseTime_ + (time - yawAccelTime_ - yawCruiseTime_) * (yawVel + yawCruiseVel_) / 2;
            Helpers::normalizeAngle(yaw);
        }

        if(!onlyYaw)
        {
            if(time < linYawAccelTime_)
            {
                linAcc = MAX_LA * 0.5;
                linVel = time * MAX_LA * 0.5;
                linDist = linVel / 2 * time;
            }
            else if(time < linYawAccelTime_ + linYawCruiseTime_)
            {
                linAcc = 0;
                linVel = linYawCruiseVel_;
                linDist = linVel / 2 * linYawAccelTime_ + linVel * (time - linYawAccelTime_);
            }
            else if (time < linYawAccelTime_ + linYawCruiseTime_ + linYawDeccelTime_)
            {
                linAcc = -(MAX_LA * 0.5);
                linVel = linYawCruiseVel_ - (MAX_LA * 0.5) * (time - linYawAccelTime_ - linYawCruiseTime_);
                linDist = linYawAccelTime_ * linYawAccelTime_ * 0.25 * MAX_LA + linYawCruiseDist_ + (time - linYawAccelTime_ - linYawCruiseTime_) * ((linVel + linYawCruiseVel_) / 2);
            }
            else
            {
                linAcc = 0;
                linVel = 0;
                linDist = actualYawDist_;
            }
        }
        else
        {
            linAcc = 0;
            linVel = 0;
            linDist = 0;
        }
        
    }
    else if (time < (yawTime + linAccelTime_ + linCruiseTime_ + linDeccelTime_))
    {
        yaw = endPose_.getYaw();
        Helpers::normalizeAngle(yaw);
        yawVel = 0;
        yawAcc = 0;

        if(time < yawTime + linAccelTime_)
        {
            linAcc = MAX_LA;
            linVel = endVel_ + MAX_LA * (time - yawTime);
            linDist = actualYawDist_ + (time - yawTime) * 0.5 * (linVel + endVel_);
        }
        else if(time < yawTime + linAccelTime_ + linCruiseTime_)
        {
            linAcc = 0;
            linVel = linCruiseVel_;
            linDist = actualYawDist_ + ((2 * endVel_ + linAccelTime_ * MAX_LA) / 2) * linAccelTime_ + (time - yawTime - linAccelTime_) * linCruiseVel_;
        }
        else
        {
            linAcc = -MAX_LA;
            linVel = linCruiseVel_ - (time - yawTime - linAccelTime_ - linCruiseTime_) * MAX_LA;
            linDist = actualYawDist_ + ((2 * endVel_ + linAccelTime_ * MAX_LA) / 2) * linAccelTime_ + linCruiseTime_ * linCruiseVel_ + (time - yawTime - linAccelTime_ - linCruiseTime_) * (linCruiseVel_ + linVel) * MAX_LA / 2;
        }
    }
    else
    {
        return endPose_;
    }

    x = startPose_.getX() + sin(ang * pi / 180) * linDist;
    xVel = sin(ang * pi / 180) * linVel;
    xAcc = sin(ang * pi / 180) * linAcc;

    y = startPose_.getY() + cos(ang * pi / 180) * linDist;
    yVel = cos(ang * pi / 180) * linVel;
    yAcc = cos(ang * pi / 180) * linAcc;

    return SwervePose(x, y, yaw, xVel, yVel, yawVel, xAcc, yAcc, yawAcc);
}

double SwerveTrajectory::getTotalTime()
{
    return yawAccelTime_ * 2 + yawCruiseTime_ + linAccelTime_ + linCruiseTime_ + linDeccelTime_;
}