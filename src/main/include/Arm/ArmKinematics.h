#pragma once

#include <iostream>
#include <tuple>

#include "Helpers/Helpers.h"
#include "ArmConstants.h"

class ArmKinematics
{
public:
	ArmKinematics();

	static std::pair<double, double> xyToAng(double x, double y, bool phiPositive);
	static std::pair<double, double> angToXY(double theta, double phi);

	static std::pair<double, double> linVelToAngVel(double xVel, double yVel, double theta, double phi);
	static std::pair<double, double> angVelToLinVel(double thetaVel, double phiVel, double theta, double phi);
private:
};

