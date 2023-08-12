#include "Arm/ArmKinematics.h"

#include <math.h>

#include "Helpers/GeometryHelper.h"

ArmKinematics::ArmKinematics()
{

}

std::pair<double, double> ArmKinematics::xyToAng(double x, double y, bool phiPositive)
{
	double radius = sqrt(x * x + y * y);
	if (x == 0 && y == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	if(radius > TwoJointArmConstants::UPPER_ARM_LENGTH + (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH))
	{
		return std::pair<double, double>{0, 0}; //TODO figure out
	}

	double phiCalc = (TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH)
	- (radius * radius)) / (2 * TwoJointArmConstants::UPPER_ARM_LENGTH * (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH));

	if(abs(phiCalc) > 1)
	{
		return std::pair<double, double>{0, 0};
	}

	double phi = acos(phiCalc);

	double returnPhi;
	if(phiPositive)
	{
		returnPhi = 180 - (phi * (180 / M_PI));
	}
	else
	{
		returnPhi = (phi * (180 / M_PI)) - 180;
	}


	double thetaCalc1 = ((TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * sin(M_PI - phi));
	if (phiPositive)
	{
		thetaCalc1 *= -1;
	}
	double thetaCalc2 = (TwoJointArmConstants::UPPER_ARM_LENGTH + (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * cos(M_PI - phi));
	if(thetaCalc1 == 0 && thetaCalc2 == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double theta = atan2(y, x) - atan2(thetaCalc1, thetaCalc2);
	theta *= (180 / M_PI);
	theta = 90 - theta;

	theta = GeometryHelper::getPrincipalAng2Deg(theta);
	phi = GeometryHelper::getPrincipalAng2Deg(returnPhi);
	return std::pair<double, double>{theta, returnPhi};
}

// Calculates endofactor coordinates given the angles of the upper arm and forearm
std::pair<double, double> ArmKinematics::angToXY(double theta, double phi)
{
	double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-theta) * (M_PI / 180));
	double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-theta) * (M_PI / 180));

	double secondJointAng_baseCords = theta + phi;

	double forearmX = -(TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * sin((-secondJointAng_baseCords) * (M_PI / 180));
	double forearmY = (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * cos((-secondJointAng_baseCords) * (M_PI / 180));

	return std::pair<double, double>{upperArmX + forearmX, upperArmY + forearmY};
}

std::pair<double, double> ArmKinematics::linVelToAngVel(double xVel, double yVel, double theta, double phi)
{
	double omega = theta + phi;
	std::pair<double, double> xy = angToXY(theta, phi);
	if(xy.first == 0 && xy.second == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double alpha = (M_PI / 2) - atan2(xy.second, xy.first);


	double linVelPhi = 0;
	double linVelTheta = 0;
	if ((theta == 0 || theta == -180) && (phi == 0 || phi == -180))
	{
		//TODO what if purely vertical
	}
	else if(omega == 90 || omega == -90)
	{
		omega *= (M_PI / 180);
		linVelPhi = (-yVel - (xVel * tan(alpha))) / (-cos(omega) * tan(alpha) + sin(omega));
		linVelTheta = (xVel - linVelPhi * cos(omega)) / cos(alpha);
	}
	else
	{
		omega *= (M_PI / 180);
		linVelTheta = (-yVel - (xVel * tan(omega))) / (-cos(alpha) * tan(omega) + sin(alpha));
		linVelPhi = (xVel - linVelTheta * cos(alpha)) / cos(omega);
	}

	double thetaRadPerSec = linVelTheta / (sqrt(xy.first * xy.first + xy.second * xy.second));
	double phiRadPerSec = linVelPhi / (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH);

	return std::pair<double, double>{thetaRadPerSec * 180 / M_PI, phiRadPerSec * 180 / M_PI};

}
std::pair<double, double> ArmKinematics::angVelToLinVel(double thetaVel, double phiVel, double theta, double phi)
{
	double forearmVel = (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * phiVel * (M_PI / 180);
	double forearmVelAng = theta + phi;
	double forearmXVel = forearmVel * cos(forearmVelAng * (M_PI / 180));
	double forearmYVel = forearmVel * -sin(forearmVelAng * (M_PI / 180));

	std::pair<double, double> xy = angToXY(theta, phi);
	double upperArmToEEDist = sqrt(xy.second * xy.second + xy.first * xy.first);
	double upperArmVel = upperArmToEEDist * thetaVel * (M_PI / 180);
	if(xy.first == 0 && xy.second == 0)
	{
		return std::pair<double, double>{0, 0};
	}
	double upperArmToEEAng = (M_PI / 2) - atan2(xy.second, xy.first);
	double upperArmXVel = upperArmVel * cos(upperArmToEEAng);
	double upperArmYVel = upperArmVel * -sin(upperArmToEEAng);

	double xVel = forearmXVel + upperArmXVel;
	double yVel = forearmYVel + upperArmYVel;
	
	return std::pair<double, double>{xVel, yVel};
}