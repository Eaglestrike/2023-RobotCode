#pragma once

#include "Constants.h"
#include <ctre/Phoenix.h>
#include <math.h>

class Arm{
    public:
        void periodic();
        void setTarget(double targetX, double targetY);

    private:
        WPI_TalonFX m_baseMotor = WPI_TalonFX(ArmConstants::BASE_MOTOR_ID);
        WPI_TalonFX m_topMotor = WPI_TalonFX(ArmConstants::TOP_MOTOR_ID);
        double m_baseArmLength = ArmConstants::BASE_ARM_LENGTH;
        double m_topArmLength = ArmConstants::TOP_ARM_LENGTH;
        double m_pivotHeight = ArmConstants::PIVOT_HEIGHT;
        double m_targetX;
        double m_targetZ;
};