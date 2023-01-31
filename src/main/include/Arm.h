#pragma once

#include "Constants.h"
#include "Helpers.h"
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include "TrajectoryManager.h"
#include "FeedForward.h"
#include "Helpers.h"

class Arm{
    public:
        void init();
        void Periodic();
        void DisabledPeriodic();
        void DisabledInit();
        void setTarget(double targetX, double targetZ);
        void moveTarget(double dx, double dz);
        void resetTarget();

    private:
        WPI_TalonFX m_baseMotor = WPI_TalonFX(ArmConstants::BASE_MOTOR_ID);
        WPI_TalonFX m_topMotor = WPI_TalonFX(ArmConstants::TOP_MOTOR_ID);

        const bool debug = true;

        const bool configDimensions = false;
        double m_baseArmLength = ArmConstants::BASE_ARM_LENGTH;
        double m_topArmLength = ArmConstants::TOP_ARM_LENGTH;
        double m_pivotHeight = ArmConstants::PIVOT_HEIGHT;

        double m_angOffsetBase = ArmConstants::BASE_OFFSET;//Radians
        double m_angOffsetTop = ArmConstants::TOP_OFFSET;//Radians

        double m_targetX = 0.0;
        double m_targetZ = m_baseArmLength + m_topArmLength + m_pivotHeight;

        const bool configPID = true;
        frc2::PIDController m_pidBase{ 
                                        ArmConstants::BASE_PID[0],
                                        ArmConstants::BASE_PID[1], 
                                        ArmConstants::BASE_PID[2]
                                    };
        double m_kGravityBot = ArmConstants::BASE_KGRAVITY;
        frc2::PIDController m_pidTop{
                                        ArmConstants::TOP_PID[0], 
                                        ArmConstants::TOP_PID[1],
                                        ArmConstants::TOP_PID[2]
                                    };
        double m_kGravityTop = ArmConstants::TOP_KGRAVITY;
        double m_maxVolts = ArmConstants::MAX_VOLTS;
		
		TrajectoryManager trajManager;
        FeedForward ff;
};