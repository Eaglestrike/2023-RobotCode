#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include "Controls.h"
#include "Constants.h"
#include "Helpers.h"
#include "TrajectoryCalc.h"
#include "TwoJointArmProfiles.h"
#include "ArmKinematics.h"
#include "Claw.h"
#include "TalonFXSim.h"

class TwoJointArm
{
    public:
        TwoJointArm();
        enum State
        {
            HOLDING_POS,
            FOLLOWING_TASK_SPACE_PROFILE,
            FOLLOWING_JOINT_SPACE_PROFILE,
            HOMING,
            STOPPED,
            MANUAL
        };
        State getState();

        void periodic();
        void zeroArms();
        void setPosTo(TwoJointArmProfiles::Positions setPosition);
        void toggleForward();
        void manualControl(double thetaVel, double phiVel);
        void stop();
        void resetIntaking();
        void switchDirections();
        void intake();
        void placeCone();
        void placeCube();

        double getTheta();
        double getPhi();
        double getThetaVel();
        double getPhiVel();

        bool posUnknown();
        bool shoulderBrakeEngaged();
        bool elbowBrakeEngaged();
        bool isForward();

        void checkPos();

        string getStateString();
        string getPosString();

        void goToPos(double thetaPos, double phiPos); //HERE
        double getThetaVolts();
        double getPhiVolts();

    private:
        // WPI_TalonFX shoulderMaster_;
        // WPI_TalonFX shoulderSlave_;
        // WPI_TalonFX elbowMaster_;
        // WPI_TalonFX elbowSlave_;
        WPI_TalonSRX shoulderMaster_;
        WPI_TalonSRX shoulderSlave_;
        WPI_TalonSRX elbowMaster_;
        WPI_TalonSRX elbowSlave_;
        // TalonFXSim shoulderMaster_;
        // TalonFXSim shoulderSlave_;
        // TalonFXSim elbowMaster_;
        // TalonFXSim elbowSlave_;
        
        frc::Solenoid shoulderBrake_;
        frc::Solenoid elbowBrake_;

        Claw claw_;

        frc::Timer timer_, clawTimer_;
        bool clawTimerStarted_;
        double taskSpaceStartTime_;

        TrajectoryCalc shoulderTraj_, elbowTraj_;

        TwoJointArmProfiles movementProfiles_;
        ArmKinematics armKinematics_;

        TwoJointArmProfiles::Positions position_, setPosition_;
        pair<TwoJointArmProfiles::Positions, TwoJointArmProfiles::Positions> key_;

        State state_;

        bool posUnknown_, shoulderBrakeEngaged_, elbowBrakeEngaged_, forward_, switchingDirections_, intaking_, gettingCone_, gotCone_;
        pair<bool, bool> homing_;
        double prevShoulderVolts_, prevElbowVolts_;
        double prevElbowPos_, elbowVel_, elbowVelTime_, prevShoulderPos_, shoulderVel_, shoulderVelTime_;

        double calcShoulderVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, bool hasCone); 
        double calcElbowVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, double hasCone);

        double calcShoulderRotInertia(double phi, bool hasCone);
        double calcElbowRotInertia(bool hasCone);

        double calcShoulderGravityTorque(double theta, double phi, bool hasCone);
        double calcElbowGravityTorque(double theta, double phi, bool hasCone);

        double calcKT(double volts);
        double calcR(double volts);

        void setShoulderVolts(double volts);
        void setElbowVolts(double volts);

        void followTaskSpaceProfile(double time);//COULDO combine into one function to calculate
        void followJointSpaceProfile();
        void home();

        void setBrakes(bool shoulder, bool elbow);

        double setPhiPos_, setThetaPos_; //HERE
};