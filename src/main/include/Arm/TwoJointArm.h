#pragma once

#include <iostream>
#include <math.h>
#include <string.h>

#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/Encoder.h>
#include <frc/DutyCycleEncoder.h>

#include "Controls/Controls.h"
#include "Helpers/Helpers.h"
#include "Helpers/TrajectoryCalc.h"
#include "Sim/TalonFXSim.h"
#include "Drivebase/SwerveConstants.h"

#include "ArmConstants.h"
#include "TwoJointArmProfiles.h"
#include "ArmKinematics.h"
#include "Claw.h"

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
        TwoJointArmProfiles::Positions getPosition();

        void periodic();
        void zeroArms();
        void zeroArmsToAutoStow();
        void reset();
        void setPosTo(TwoJointArmProfiles::Positions setPosition);
        void specialSetPosTo(TwoJointArmProfiles::Positions setPosition);
        void toggleForward();
        // void toggleForwardCubeIntake();
        void toggleForwardExtendedToCubeIntake();
        void manualControl(double thetaVel, double phiVel, bool gravity);
        void stop();
        // void resetIntaking();
        // void intake();
        // void placeCone();
        // void placeCube();

        void setJointPath(double theta, double phi);

        double getTheta();
        double getPhi();
        double getThetaVel();
        double getPhiVel();

        bool posUnknown();
        bool shoulderBrakeEngaged();
        bool elbowBrakeEngaged();
        bool isForward();
        // bool intaking();

        void checkPos();

        std::string getStateString();
        std::string getPosString();
        std::string getSetPosString();

        // void goToPos(double thetaPos, double phiPos); //HERE
        // double getThetaVolts();
        // double getPhiVolts();

        bool getClawOpen();
        void setClaw(bool open);
        void setClawWheels(double speed);
        void setEStopped(bool eStopped);
        bool isEStopped();
        std::pair<bool, bool> intakesNeededDown();
        void updateIntakeStates(bool cubeIntakeDown, bool coneIntakeDown);
        void setForward(bool forward);

        double getClawWheelSpeed();
        bool clawOpen();

    private:
        WPI_TalonFX shoulderMaster_;
        WPI_TalonFX shoulderSlave_;
        WPI_TalonFX elbowMaster_;
        WPI_TalonFX elbowSlave_;
        // WPI_TalonSRX shoulderMaster_;
        // WPI_TalonSRX shoulderSlave_;
        // WPI_TalonSRX elbowMaster_;
        // WPI_TalonSRX elbowSlave_;
        // TalonFXSim shoulderMaster_;
        // TalonFXSim shoulderSlave_;
        // TalonFXSim elbowMaster_;
        // TalonFXSim elbowSlave_;
        
        frc::Solenoid shoulderBrake_;
        frc::Solenoid elbowBrake_;

        frc::DutyCycleEncoder shoulderEncoder_;

        Claw claw_;

        frc::Timer timer_, clawTimer_;
        //bool clawTimerStarted_;
        double taskSpaceStartTime_;

        TrajectoryCalc shoulderTraj_, elbowTraj_;

        TwoJointArmProfiles movementProfiles_;
        ArmKinematics armKinematics_;

        TwoJointArmProfiles::Positions position_, setPosition_;
        std::pair<TwoJointArmProfiles::Positions, TwoJointArmProfiles::Positions> key_;

        State state_;

        bool posUnknown_, shoulderBrakeEngaged_, elbowBrakeEngaged_, forward_, switchingDirections_, /*intaking_, gettingCone_, 
        gotCone_,*/ eStopped_, cubeIntakeNeededDown_, coneIntakeNeededDown_, switchingToCubeIntake_, ciSwitchFirstStageDone_, cubeIntakeDown_, coneIntakeDown_;
        std::pair<bool, bool> homing_;
        std::pair<bool, bool> homingFirstStage_;
        std::pair<bool, bool> homingSecondStage_;
        bool homingRaising_;
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

        bool setShoulderVolts(double volts);
        bool setElbowVolts(double volts);

        void followTaskSpaceProfile(double time);//COULDO combine into one function to calculate
        void followJointSpaceProfile();
        void home();
        void homeNew();

        void setBrakes(bool shoulder, bool elbow);

        // void switchDirections();
        // void switchDirectionsCubeIntake();
        void swingthroughExtendedToCubeIntake();

        // double setPhiPos_, setThetaPos_; //HERE
};