#pragma once
#include <tuple>
#include <map>
#include <iostream>
#include <fstream>
#include <string>

#include "frc/Filesystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

class TwoJointArmProfiles
{
public:
	enum Positions
	{
		STOWED,
		CUBE_INTAKE,
		MID,
		SPECIAL,
		HIGH,
		CUBE_MID,
		CUBE_HIGH,
		GROUND,
		RAMMING_PLAYER_STATION,
		AUTO_STOW/*,
		CONE_INTAKE*/
	};
	TwoJointArmProfiles();

	void readProfiles();

	std::tuple<double, double, double> getThetaProfile(std::pair<Positions, Positions>key , double time);
	std::tuple<double, double, double> getPhiProfile(std::pair<Positions, Positions> key, double time);

private:
	std::map<std::pair<Positions, Positions>, std::map<double, std::pair<std::tuple<double, double, double>, std::tuple<double, double, double>>>> profiles_; //Ok this is big but it makes the most sense at least to me

	bool hasProfiles_;
};

