#include "SwervePose.h"

SwervePose::SwervePose(double x, double y, double yaw, double xVel, double yVel, double yawVel, double xAcc, double yAcc, double yawAcc) : x_(x), y_(y), yaw_(yaw), yawDist_(0), xVel_(xVel), yVel_(yVel), yawVel_(yawVel), xAcc_(xAcc), yAcc_(yAcc), yawAcc_(yawAcc) 
{

}

SwervePose::SwervePose(double x, double y, double yaw, double yawDist)  : x_(x), y_(y), yaw_(yaw), yawDist_(yawDist), xVel_(0), yVel_(0), yawVel_(0), xAcc_(0), yAcc_(0), yawAcc_(0) 
{

}

double SwervePose::getX()
{
    return x_;
}

double SwervePose::getY()
{
    return y_;
}

double SwervePose::getYaw()
{
    return yaw_;
}

double SwervePose::getYawDist()
{
    return yawDist_;
}

double SwervePose::getXVel()
{
    return xVel_;
}

double SwervePose::getYVel()
{
    return yVel_;
}

double SwervePose::getYawVel()
{
    return yawVel_;
}

double SwervePose::getXAcc()
{
    return xAcc_;
}

double SwervePose::getYAcc()
{
    return yAcc_;
}

double SwervePose::getYawAcc()
{
    return yawAcc_;
}

double SwervePose::distTo(SwervePose endPose)
{
    double dx = endPose.getX() - x_;
    double dy = endPose.getY() - y_;
    return sqrt(dx*dx + dy*dy);
}

double SwervePose::angTo(SwervePose endPose)
{
    double dx = endPose.getX() - x_;
    double dy = endPose.getY() - y_;
    if(dx != 0 || dy != 0)
    {
        return 90 - (atan2(dy, dx) * 180 / M_PI);
    }
    else
    {
        return 0;
    }
}