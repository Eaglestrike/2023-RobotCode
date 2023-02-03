#pragma once

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"

class TalonFXSim
{
    public:
        TalonFXSim(int id) : id_(id), pos_(0), vel_(0), acc_(0)
        {

        }

        void SetVoltage(units::volt_t volts)
        {
            volts_ = volts.value();
        }
        void SetSelectedSensorPosition(double pos)
        {
            pos_ = pos;
        }
        double GetSelectedSensorVelocity()
        {
            return vel_;
        }
        double GetSelectedSensorPosition()
        {
            return pos_;
        }

        void setVel(double vel)
        {
            vel_ = vel;
        }
        void setPos_(double pos)
        {
            pos_ = pos;
        }
        double getVolts()
        {
            return volts_;
        }
    private:
        int id_;
        double pos_, vel_, acc_, volts_;
};