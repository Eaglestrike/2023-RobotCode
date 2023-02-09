#pragma once

#include "Constants.h"
#include "Helpers.h"
#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <iostream>
#include <frc/Solenoid.h>
#include <frc/DutyCycleEncoder.h>

class Arm {
    public:
        void init();
        void Periodic();
        void TeleopPeriodic();
        void DisabledPeriodic();
        void DisabledInit();
        void TestPeriodic(double vBase, double vTop);
        void setTarget(double targetX, double targetZ);
        void moveTarget(double dx, double dz);
        void changeOffsetBase(double da);
        void changeOffsetTop(double da);
        void resetTarget();
        void resetEncoder();
        void resetOffsets();

    private:
        void ReadSmartDashboard();

        WPI_TalonFX m_baseMotor = WPI_TalonFX(ArmConstants::BASE_MOTOR_ID);
        WPI_TalonFX m_topMotor = WPI_TalonFX(ArmConstants::TOP_MOTOR_ID);
        WPI_TalonFX m_baseMotor2 = WPI_TalonFX(ArmConstants::BASE_MOTOR_ID2);
        WPI_TalonFX m_topMotor2 = WPI_TalonFX(ArmConstants::TOP_MOTOR_ID2);

        frc::DutyCycleEncoder baseEncoder{2};
        
        frc::Solenoid m_baseBrake = frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, ArmConstants::BASE_BRAKE_ID);
        frc::Solenoid m_topBrake = frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, ArmConstants::TOP_BRAKE_ID);
        void setBrakes(bool base, bool top);
        //Supply Voltage using gravity
        void TargetFail();

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

        double m_botArmSlack = ArmConstants::BOT_ARM_SLACK;//Radians
        double m_topArmSlack = ArmConstants::TOP_ARM_SLACK;//Radians
                        
        double m_kGravityTop = ArmConstants::TOP_KGRAVITY;
        double m_maxVolts = ArmConstants::MAX_VOLTS;

        //Updates in periodic
        double baseReading = 0.0;
        double topReading = 0.0;

};