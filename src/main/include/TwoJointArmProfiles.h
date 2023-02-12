#pragma once
#include <tuple>
#include <map>
#include "constants.h"
#include <iostream>
#include <fstream>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>

class TwoJointArmProfiles
{
public:
	enum Positions
	{
		STOWED,
		CUBE_INTAKE,
		PLAYER_STATION,
		MID,
		HIGH,
		CUBE_MID,
		CUBE_HIGH,
		CONE_INTAKE
	};
	TwoJointArmProfiles();

	void readProfiles();

	tuple<double, double, double> getThetaProfile(pair<Positions, Positions>key , double time);
	tuple<double, double, double> getPhiProfile(pair<Positions, Positions> key, double time);

private:
	map<pair<Positions, Positions>, map<double, pair<tuple<double, double, double>, tuple<double, double, double>>>> profiles_; //Ok this is big but it makes the most sense at least to me

	bool hasProfiles_;
};

