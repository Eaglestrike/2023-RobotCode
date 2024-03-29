#include "Arm/TwoJointArm.h"

TwoJointArm::TwoJointArm() : shoulderMaster_(TwoJointArmConstants::SHOULDER_MASTER_ID), shoulderSlave_(TwoJointArmConstants::SHOULDER_SLAVE_ID),
                             elbowMaster_(TwoJointArmConstants::ELBOW_MASTER_ID), elbowSlave_(TwoJointArmConstants::ELBOW_SLAVE_ID),
                             shoulderBrake_(frc::PneumaticsModuleType::CTREPCM, TwoJointArmConstants::SHOULDER_BRAKE_ID), elbowBrake_(frc::PneumaticsModuleType::CTREPCM, TwoJointArmConstants::ELBOW_BRAKE_ID), shoulderEncoder_(TwoJointArmConstants::SHOULDER_ENCODER_ID),
                             shoulderTraj_(TwoJointArmConstants::SHOULDER_ARM_MAX_VEL, TwoJointArmConstants::SHOULDER_ARM_MAX_ACC, 0, 0, 0, 0), elbowTraj_(TwoJointArmConstants::ELBOW_ARM_MAX_VEL, TwoJointArmConstants::ELBOW_ARM_MAX_ACC, 0, 0, 0, 0)
{
    shoulderMaster_.SetNeutralMode(NeutralMode::Brake);
    shoulderMaster_.SetInverted(true);
    shoulderSlave_.SetNeutralMode(NeutralMode::Brake);
    shoulderSlave_.Follow(shoulderMaster_);
    shoulderSlave_.SetInverted(InvertType::FollowMaster);

    elbowMaster_.SetNeutralMode(NeutralMode::Brake);
    elbowSlave_.SetNeutralMode(NeutralMode::Brake);
    elbowMaster_.SetInverted(true);
    elbowSlave_.Follow(elbowMaster_);
    elbowSlave_.SetInverted(InvertType::FollowMaster);

    claw_.setOpen(false);

    movementProfiles_.readProfiles();
    state_ = STOPPED;
    position_ = TwoJointArmProfiles::STOWED;
    setPosition_ = TwoJointArmProfiles::STOWED;
    key_ = {TwoJointArmProfiles::STOWED, TwoJointArmProfiles::STOWED};
    posUnknown_ = true;
    // zeroArmsToStow();
    zeroArms();
    stop();
    forward_ = true;
    homing_ = {false, false};
    homingFirstStage_ = {false, false};
    homingSecondStage_ = {false, false};
    homingRaising_ = false;
    switchingDirections_ = false;
    switchingToCubeIntake_ = false;
    ciSwitchFirstStageDone_ = false;
    // intaking_ = false;
    // gettingCone_ = false;
    // gotCone_ = false;
    // clawTimerStarted_ = false;
    setBrakes(true, true);
    taskSpaceStartTime_ = 0;
    eStopped_ = false;
    cubeIntakeNeededDown_ = false;
    coneIntakeNeededDown_ = false;

    prevElbowPos_ = 0;
    elbowVel_ = 0;
    prevShoulderPos_ = 0;
    shoulderVel_ = 0;
    elbowVelTime_ = 0;
    shoulderVelTime_ = 0;
    elbowTraj_.generateTrajectory(0, 0, 0);
    elbowTraj_.generateTrajectory(0, 0, 0);

    prevShoulderVolts_ = 0;
    prevElbowVolts_ = 0;
}

TwoJointArm::State TwoJointArm::getState()
{
    return state_;
}

TwoJointArmProfiles::Positions TwoJointArm::getPosition()
{
    return position_;
}

void TwoJointArm::periodic()
{
    if (eStopped_)
    {
        claw_.setWheelSpeed(0);
        state_ = STOPPED;
    }

    claw_.periodic();
    // frc::SmartDashboard::PutBoolean("Intaking", intaking_);

    switch (state_)
    {
    case HOLDING_POS:
    {
        // if (!forward_ && position_ == TwoJointArmProfiles::CUBE_INTAKE)
        // {
        //     setClawWheels(ClawConstants::INTAKING_SPEED);
        // }
        // else if (!forward_)
        // {
        //     setClawWheels(0);
        // }
        checkPos();
        if (posUnknown_)
        {
            state_ = STOPPED;
        }
        stop();
        if (position_ != setPosition_)
        {
            if (setPosition_ == TwoJointArmProfiles::SPECIAL)
            {
                setPosition_ = position_;
                if (position_ == TwoJointArmProfiles::SPECIAL)
                {
                    state_ = STOPPED;
                    position_ = TwoJointArmProfiles::STOWED;
                    setPosition_ = TwoJointArmProfiles::STOWED;
                }
            }
            else
            {
                setPosTo(setPosition_);
            }
        }

        cubeIntakeNeededDown_ = ((position_ == TwoJointArmProfiles::CUBE_INTAKE || position_ == TwoJointArmProfiles::GROUND) && !forward_);
        coneIntakeNeededDown_ = ((position_ == TwoJointArmProfiles::GROUND) && forward_);
        break;
    }
    case FOLLOWING_TASK_SPACE_PROFILE:
    {
        setBrakes(false, false);
        followTaskSpaceProfile(timer_.GetFPGATimestamp().value() - taskSpaceStartTime_);
        break;
    }
    case FOLLOWING_JOINT_SPACE_PROFILE:
    {
        // TODO figure out intakes
        setBrakes(false, false);
        followJointSpaceProfile();
        break;
    }
    case HOMING:
    {
        // setBrakes(false);
        homeNew();
        break;
    }
    case STOPPED:
    {
        // Intakes do nothing stop panicking
        stop();
        // resetIntaking();
        checkPos();
        // setPosition_ = position_;
        posUnknown_ = true;
        checkPos();
        setPosition_ = position_;
        break;
    }
    case MANUAL:
    {
        // setBrakes(false, false);
        posUnknown_ = true;
        break;
    }
    }

    // if (intaking_)
    // {
    //     intake();
    // }
}

void TwoJointArm::zeroArms()
{
    shoulderMaster_.SetSelectedSensorPosition(0);
    double theta = getTheta();
    if (!forward_)
    {
        theta *= -1;
    }
    double elbowPos = (180 + (theta * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO)) * 2048 / 360.0 / TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO / TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    elbowMaster_.SetSelectedSensorPosition(elbowPos);

    // shoulderMaster_.setPos_(0);
    // elbowMaster_.setPos_(180);
}

void TwoJointArm::zeroArmsToAutoStow()
{
    //-18.5, 164.5
    double shoulderAng = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][2];
    double elbowAng = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][3];

    double shoulderPos = shoulderAng * 2048.0 / 360.0 / TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    shoulderMaster_.SetSelectedSensorPosition(shoulderPos);
    // shoulderMaster_.setPos_(-18.5);

    // double elbowAng = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3] + TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2] - (getTheta());
    // double elbowPos = (elbowAng + (getTheta() * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO)) * 2048 / 360.0 / TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO / TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double theta = getTheta();
    if (!forward_)
    {
        theta *= -1;
    }
    double elbowPos = (elbowAng + (theta * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO)) * 2048 / 360.0 / TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO / TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    elbowMaster_.SetSelectedSensorPosition(elbowPos);
    // elbowMaster_.setPos_(164.5);

    stop();
    forward_ = true;

    position_ = TwoJointArmProfiles::AUTO_STOW;
    setPosition_ = TwoJointArmProfiles::AUTO_STOW;
    state_ = HOLDING_POS;
}

void TwoJointArm::reset()
{
    position_ = TwoJointArmProfiles::STOWED;
    setPosition_ = TwoJointArmProfiles::STOWED;
    coneIntakeNeededDown_ = false;
    cubeIntakeNeededDown_ = false;
    posUnknown_ = false;
    switchingDirections_ = false;
    switchingToCubeIntake_ = false;
    ciSwitchFirstStageDone_ = false;
    // intaking_ = false;
    // gettingCone_ = false;
    // gotCone_ = false;
    cubeIntakeNeededDown_ = false;
    coneIntakeNeededDown_ = false;

    homing_ = {false, false};
    homingFirstStage_ = {false, false};
    homingSecondStage_ = {false, false};
    homingRaising_ = false;
    switchingDirections_ = false;
    switchingToCubeIntake_ = false;
    ciSwitchFirstStageDone_ = false;
}

void TwoJointArm::setPosTo(TwoJointArmProfiles::Positions setPosition)
{
    if (switchingDirections_)
    {
        return;
    }
    if (state_ == FOLLOWING_TASK_SPACE_PROFILE || state_ == FOLLOWING_JOINT_SPACE_PROFILE)
    {
        return;
    }
    if (posUnknown_)
    {
        checkPos();
    }
    // if(setPosition == TwoJointArmProfiles::SPECIAL)
    // {
    //     return;
    // }
    // if(position_ == TwoJointArmProfiles::SPECIAL)
    // {
    //     return;
    // }
    // if(position_ == TwoJointArmProfiles::AUTO_STOW)
    // {
    //     if(setPosition == TwoJointArmProfiles::STOWED || setPosition == TwoJointArmProfiles::CUBE_INTAKE || setPosition == TwoJointArmProfiles::GROUND || setPosition == TwoJointArmProfiles::RAMMING_PLAYER_STATION)
    //     {
    //         return;
    //     }
    // }
    // if(setPosition == TwoJointArmProfiles::AUTO_STOW)
    // {
    //     if(position_ == TwoJointArmProfiles::STOWED || position_ == TwoJointArmProfiles::CUBE_INTAKE || position_ == TwoJointArmProfiles::GROUND || position_ == TwoJointArmProfiles::RAMMING_PLAYER_STATION)
    //     {
    //         return;
    //     }
    // }

    bool atGroundPos = (abs(getTheta()) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(getPhi() - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD);
    if (state_ == STOPPED && !atGroundPos)
    {
        state_ = HOMING;
    }

    if (state_ == HOMING)
    {
        setPosition_ = setPosition;
        return;
    }

    if (position_ == setPosition && !atGroundPos)
    {
        return;
    }

    // if (position_ == TwoJointArmProfiles::CONE_INTAKE && setPosition != TwoJointArmProfiles::STOWED)
    // {
    //     return;
    // }

    // if (setPosition == TwoJointArmProfiles::CONE_INTAKE && position_ != TwoJointArmProfiles::STOWED)
    // {
    //     return;
    // }


    if(atGroundPos)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::AUTO_STOW && setPosition == TwoJointArmProfiles::GROUND)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        setPosition_ = setPosition;
    }
    else if (position_ == TwoJointArmProfiles::AUTO_STOW && setPosition == TwoJointArmProfiles::STOWED)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::STOWED && setPosition == TwoJointArmProfiles::AUTO_STOW)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::AUTO_STOW && setPosition == TwoJointArmProfiles::RAMMING_PLAYER_STATION)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::RAMMING_PLAYER_STATION && setPosition == TwoJointArmProfiles::AUTO_STOW)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::STOWED && setPosition == TwoJointArmProfiles::RAMMING_PLAYER_STATION) //From here COMP
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
    else if (position_ == TwoJointArmProfiles::RAMMING_PLAYER_STATION && setPosition == TwoJointArmProfiles::STOWED)
    {
        shoulderTraj_.generateTrajectory(getTheta(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        elbowTraj_.generateTrajectory(getPhi(), TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }// To here, comment out for curved path from stow to ramming ps
    else if (position_ == TwoJointArmProfiles::GROUND)
    {
        key_ = {position_, TwoJointArmProfiles::STOWED};

        setPosition_ = setPosition;
        state_ = FOLLOWING_TASK_SPACE_PROFILE;
        taskSpaceStartTime_ = timer_.GetFPGATimestamp().value();
    }
    else
    {
        key_ = {position_, setPosition};

        setPosition_ = setPosition;
        state_ = FOLLOWING_TASK_SPACE_PROFILE;
        taskSpaceStartTime_ = timer_.GetFPGATimestamp().value();
    }
}

void TwoJointArm::specialSetPosTo(TwoJointArmProfiles::Positions setPosition)
{
    if (position_ != TwoJointArmProfiles::CUBE_INTAKE)
    {
        return;
    }

    if (forward_)
    {
        return;
    }

    if (switchingDirections_)
    {
        return;
    }
    if (posUnknown_)
    {
        return;
    }

    if (state_ == STOPPED)
    {
        return;
    }

    if (state_ == HOMING)
    {
        setPosition_ = position_;
        return;
    }

    if (position_ == setPosition)
    {
        return;
    }

    if (state_ == FOLLOWING_TASK_SPACE_PROFILE)
    {
        return;
    }

    /*if(!forward_ && setPosition == TwoJointArmProfiles::ABOVE_INTAKE)
    {
        if(position_ == TwoJointArmProfiles::STOWED)
        {
            forward_ = true;
            key_ = {position_, TwoJointArmProfiles::ABOVE_INTAKE};
        }
        else
        {
            claw_.setOpen(false);
            key_ = {position_, TwoJointArmProfiles::STOWED};
        }

    }
    else
    {
        if(setPosition == TwoJointArmProfiles::STOWED)
        {
            claw_.setOpen(false);
        }
        key_ = {position_, setPosition};
    }*/

    forward_ = true;
    position_ = TwoJointArmProfiles::SPECIAL;
    key_ = {position_, setPosition};

    setPosition_ = setPosition;
    state_ = FOLLOWING_TASK_SPACE_PROFILE;
    taskSpaceStartTime_ = timer_.GetFPGATimestamp().value();
}

void TwoJointArm::setBrakes(bool shoulder, bool elbow)
{
    shoulderBrake_.Set(!shoulder);
    elbowBrake_.Set(!elbow);
    shoulderBrakeEngaged_ = shoulder;
    elbowBrakeEngaged_ = elbow;
}
void TwoJointArm::stop()
{
    if (state_ == FOLLOWING_JOINT_SPACE_PROFILE || state_ == FOLLOWING_TASK_SPACE_PROFILE || state_ == HOMING || state_ == MANUAL)
    {
        state_ = STOPPED;
    }
    else if (state_ == HOLDING_POS)
    {
        checkPos();
    }
    setPosition_ = position_;
    setShoulderVolts(0);
    setElbowVolts(0);
    setBrakes(true, true);
    homing_ = {false, false};
    homingFirstStage_ = {false, false};
    homingSecondStage_ = {false, false};
    homingRaising_ = false;
    switchingDirections_ = false;
    switchingToCubeIntake_ = false;
    ciSwitchFirstStageDone_ = false;
    // intaking_ = false;
    // gettingCone_ = false;
    // gotCone_ = false;
}

// void TwoJointArm::resetIntaking()
// {
//     intaking_ = false;
//     gettingCone_ = false;
//     gotCone_ = false;
//     clawTimerStarted_ = false;
// }

// void TwoJointArm::switchDirections()
// {
//     double theta = getTheta();
//     double phi = getPhi();
//     double wantedTheta = -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2];
//     double wantedPhi = 360 - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3];
//     // double thetaVel = getThetaVel();
//     // double phiVel = getPhiVel();

//     shoulderTraj_.generateTrajectory(theta, wantedTheta, 0);
//     elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
//     state_ = FOLLOWING_JOINT_SPACE_PROFILE;

//     switchingDirections_ = true;
//     switchingToCubeIntake_ = false;
//     ciSwitchFirstStageDone_ = false;

//     // claw_.setOpen(false);
// }

// void TwoJointArm::switchDirectionsCubeIntake()
// {
//     double theta = getTheta();
//     double phi = getPhi();
//     double wantedTheta, wantedPhi;
//     if (forward_)
//     {
//         //wantedTheta = -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][2] + 10; // CONE PAIN set all to default and make ciSwitch true
//         wantedTheta = 10; //for 11 stow
//         wantedPhi = 360 - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][3];
//         switchingToCubeIntake_ = true;
//         ciSwitchFirstStageDone_ = false;
//     }
//     else
//     {
//         // wantedTheta = -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2];
//         // wantedPhi = 360 - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3];
//         wantedTheta = -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2] - TwoJointArmConstants::SWINGTHROUGH_CLEARANCE;
//         wantedPhi = 360 - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3];
//         switchingToCubeIntake_ = false;
//         ciSwitchFirstStageDone_ = false;
//     }

//     // double thetaVel = getThetaVel();
//     // double phiVel = getPhiVel();

//     shoulderTraj_.generateTrajectory(theta, wantedTheta, 0);
//     elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
//     state_ = FOLLOWING_JOINT_SPACE_PROFILE;

//     switchingDirections_ = true;

//     // claw_.setOpen(false);
// }

void TwoJointArm::swingthroughExtendedToCubeIntake()
{
    if (position_ == TwoJointArmProfiles::HIGH || position_ == TwoJointArmProfiles::MID || position_ == TwoJointArmProfiles::CUBE_HIGH || position_ == TwoJointArmProfiles::CUBE_MID)
    {
        setPosTo(TwoJointArmProfiles::SPECIAL);
        state_ = FOLLOWING_TASK_SPACE_PROFILE;
        switchingDirections_ = true;
    }
}

// void TwoJointArm::intake()
// {
//     if (state_ == FOLLOWING_TASK_SPACE_PROFILE || state_ == FOLLOWING_JOINT_SPACE_PROFILE || state_ == HOMING)
//     {
//         return;
//     }

//     if (state_ == MANUAL)
//     {
//         intaking_ = false;
//         gettingCone_ = false;
//         gotCone_ = false;
//         clawTimerStarted_ = false;
//         return;
//     }

//     if (!intaking_)
//     {
//         intaking_ = true;
//     }

//     if (!gettingCone_)
//     {
//         if ((position_ != TwoJointArmProfiles::STOWED || state_ == STOPPED))
//         {
//             // Not above the intake, move there
//             // stop();
//             intaking_ = true;
//             gettingCone_ = false;
//             gotCone_ = false;
//             clawTimerStarted_ = false;
//             setPosTo(TwoJointArmProfiles::STOWED);
//             state_ = FOLLOWING_TASK_SPACE_PROFILE;
//             return;
//         }
//         else if (state_ == HOLDING_POS)
//         {
//             // Above the intake, start getting the cone
//             setPosTo(TwoJointArmProfiles::CONE_INTAKE);
//             gettingCone_ = true;
//         }
//         else
//         {
//             // Moving to above the intake, wait
//             return;
//         }
//     }
//     else
//     {
//         // getting cone
//         if (state_ == HOLDING_POS && !gotCone_)
//         {
//             if (position_ == TwoJointArmProfiles::STOWED)
//             {
//                 setPosTo(TwoJointArmProfiles::CONE_INTAKE);
//                 setClawWheels(ClawConstants::INTAKING_SPEED);
//                 claw_.setOpen(true);
//             }
//             else if (position_ == TwoJointArmProfiles::CONE_INTAKE)
//             {
//                 if (!clawTimerStarted_)
//                 {
//                     clawTimer_.Reset();
//                     clawTimer_.Start();
//                     clawTimerStarted_ = true;
//                 }

//                 if (clawTimer_.Get().value() > 0.1)
//                 {
//                     claw_.setOpen(false);
//                 }

//                 if (clawTimer_.Get().value() > 0.4)
//                 {
//                     setPosTo(TwoJointArmProfiles::STOWED);
//                     gotCone_ = true;
//                     setClawWheels(0);
//                 }
//             }
//         }
//         else if (state_ == HOLDING_POS && gotCone_)
//         {
//             // Done
//             intaking_ = false;
//             gettingCone_ = false;
//             gotCone_ = false;
//             clawTimerStarted_ = false;
//         }
//     }
// }

// void TwoJointArm::placeCone()
// {
//     if (state_ != HOLDING_POS && state_ != MANUAL && state_ != STOPPED)
//     {
//         return;
//     } // Driver preference

//     // if(position_ == TwoJointArmProfiles::HIGH || position_ == TwoJointArmProfiles::MID)
//     //{
//     claw_.setOpen(true);
//     //}//Driver preference
// }

// void TwoJointArm::placeCube()
// {
//     setClawWheels(ClawConstants::OUTAKING_SPEED);
//     claw_.setOpen(true);
// }

void TwoJointArm::setJointPath(double theta, double phi)
{
    if (state_ == HOLDING_POS || state_ == STOPPED)
    {
        shoulderTraj_.generateTrajectory(getTheta(), theta, 0);
        elbowTraj_.generateTrajectory(getPhi(), phi, 0);
        state_ = FOLLOWING_JOINT_SPACE_PROFILE;
    }
}

void TwoJointArm::followTaskSpaceProfile(double time)
{
    std::tuple<double, double, double> thetaProfile = movementProfiles_.getThetaProfile(key_, time);
    std::tuple<double, double, double> phiProfile = movementProfiles_.getPhiProfile(key_, time);

    double wantedTheta = get<0>(thetaProfile);
    double wantedPhi = get<0>(phiProfile);
    double wantedThetaVel = get<1>(thetaProfile);
    double wantedPhiVel = get<1>(phiProfile);
    double wantedThetaAcc = get<2>(thetaProfile);
    double wantedPhiAcc = get<2>(phiProfile);

    // frc::SmartDashboard::PutNumber("WTheta", wantedTheta);
    // frc::SmartDashboard::PutNumber("WPhi", wantedPhi);

    double theta = getTheta();
    double phi = getPhi();
    // double shoulderVel = getThetaVel();
    // double elbowVel = getPhiVel();

    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);

    bool intakeNeededDown;
    double intakeLength, collisionBuffer;
    if (!forward_ || key_.first == TwoJointArmProfiles::SPECIAL)
    {
        xy.first -= TwoJointArmConstants::CUBE_INTAKE_TO_SHOULDER_X;
        xy.second += TwoJointArmConstants::CUBE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        intakeLength = TwoJointArmConstants::CUBE_INTAKE_LENGTH;
        collisionBuffer = TwoJointArmConstants::CUBE_INTAKE_COLLISION_BUFFER;
    }
    else
    {
        xy.first -= TwoJointArmConstants::CONE_INTAKE_TO_SHOULDER_X;
        xy.second += TwoJointArmConstants::CONE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        intakeLength = TwoJointArmConstants::CONE_INTAKE_LENGTH;
        collisionBuffer = TwoJointArmConstants::CONE_INTAKE_COLLISION_BUFFER;
    }

    if (xy.first < 0)
    {
        std::tuple<double, double, double> thetaAheadProfile = movementProfiles_.getThetaProfile(key_, time + TwoJointArmConstants::COLLISION_LOOKAHEAD_TIME);
        std::tuple<double, double, double> phiAheadProfile = movementProfiles_.getPhiProfile(key_, time + TwoJointArmConstants::COLLISION_LOOKAHEAD_TIME);
        double aheadTheta = get<0>(thetaAheadProfile);
        double aheadPhi = get<0>(phiAheadProfile);
        std::pair<double, double> xyAhead = ArmKinematics::angToXY(aheadTheta, aheadPhi);

        if (xyAhead.first > 0)
        {
            intakeNeededDown = true;
        }
        else
        {
            intakeNeededDown = false;
        }
    }
    else
    {
        if (sqrt(xy.first * xy.first + xy.second * xy.second) < intakeLength + collisionBuffer)
        {
            intakeNeededDown = true;
        }
        else
        {
            std::tuple<double, double, double> thetaAheadProfile = movementProfiles_.getThetaProfile(key_, time + TwoJointArmConstants::COLLISION_LOOKAHEAD_TIME);
            std::tuple<double, double, double> phiAheadProfile = movementProfiles_.getPhiProfile(key_, time + TwoJointArmConstants::COLLISION_LOOKAHEAD_TIME);
            double aheadTheta = get<0>(thetaAheadProfile);
            double aheadPhi = get<0>(phiAheadProfile);
            std::pair<double, double> xyAhead = ArmKinematics::angToXY(aheadTheta, aheadPhi);

            if (xyAhead.first < 0 && xyAhead.second < intakeLength) // probably not even needed
            {
                intakeNeededDown = true;
            }
            else if (sqrt(xyAhead.first * xyAhead.first + xyAhead.second * xyAhead.second) < intakeLength + collisionBuffer)
            {
                intakeNeededDown = true;
            }
            else
            {
                intakeNeededDown = false;
            }
        }
    }

    if (forward_)
    {
        coneIntakeNeededDown_ = intakeNeededDown;
        if(key_.first == TwoJointArmProfiles::SPECIAL)
        {
            cubeIntakeNeededDown_ = true;
        }
        else
        {
            cubeIntakeDown_ = false;
        }
    }
    else
    {
        coneIntakeNeededDown_ = false;
        cubeIntakeNeededDown_ = intakeNeededDown;
    }

    if (wantedThetaVel == 0 && wantedPhiVel == 0 && wantedThetaAcc == 0 && wantedPhiAcc == 0)
    {
        if (switchingDirections_)
        {
            forward_ = !forward_;
            switchingDirections_ = false;

            wantedTheta = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][2];
            wantedPhi = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][3];
            switchingToCubeIntake_ = false;
            setPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
            position_ = TwoJointArmProfiles::CUBE_INTAKE;

            theta = getTheta();
            phi = getPhi();
        }

        position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
        {
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD || abs(phi - wantedPhi) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
        {
            shoulderTraj_.generateTrajectory(theta, wantedTheta, 0); // shoulder vel was bad
            elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            state_ = HOLDING_POS;
        }
    }

    wantedPhiVel += wantedThetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    wantedPhiAcc += wantedThetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    // frc::SmartDashboard::PutNumber("WTVel", wantedThetaVel);
    // frc::SmartDashboard::PutNumber("WPVel", wantedPhiVel);
    // frc::SmartDashboard::PutNumber("WTPos", wantedTheta);
    // frc::SmartDashboard::PutNumber("WPPos", wantedPhi);
    double thetaVolts = calcShoulderVolts(wantedThetaVel, wantedThetaAcc, wantedTheta, theta, phi, false);
    double phiVolts = calcElbowVolts(wantedPhiVel, wantedPhiAcc, wantedPhi, theta, phi, false);

    setShoulderVolts(thetaVolts);
    // shoulderMaster_.setVel(wantedThetaVel);
    // shoulderMaster_.setPos_(wantedTheta);
    setElbowVolts(phiVolts);
    // elbowMaster_.setVel(wantedPhiVel);
    // elbowMaster_.setPos_(wantedPhi);
}

void TwoJointArm::followJointSpaceProfile()
{
    std::tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile();
    std::tuple<double, double, double> elbowProfile = elbowTraj_.getProfile();

    double wantedTheta = get<2>(shoulderProfile);
    double wantedPhi = get<2>(elbowProfile);
    double wantedThetaVel = get<1>(shoulderProfile);
    double wantedPhiVel = get<1>(elbowProfile);
    double wantedThetaAcc = get<0>(shoulderProfile);
    double wantedPhiAcc = get<0>(elbowProfile);

    double theta = getTheta();
    double phi = getPhi();
    // double shoulderVel = getThetaVel();
    // double elbowVel = getPhiVel();

    if (switchingDirections_ && !ciSwitchFirstStageDone_ && wantedThetaVel == 0 && wantedThetaAcc == 0)
    {

        if (switchingToCubeIntake_) // no if just stowed num generateTrajectory for different swingthrough
        {
            // if(abs(phi - 180) < 3) //DIFFERENT SWINGTHROUGH
            // {
            shoulderTraj_.generateTrajectory(theta, -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][2], 0);
            ciSwitchFirstStageDone_ = true;
            // }
        }
        else
        {
            ciSwitchFirstStageDone_ = true;
            shoulderTraj_.generateTrajectory(theta, -TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        }

        shoulderProfile = shoulderTraj_.getProfile();
        wantedTheta = get<2>(shoulderProfile);
        wantedThetaVel = get<1>(shoulderProfile);
        wantedThetaAcc = get<0>(shoulderProfile);
    }

    if (wantedThetaVel == 0 && wantedPhiVel == 0 && wantedThetaAcc == 0 && wantedPhiAcc == 0)
    {
        if (switchingDirections_ && ciSwitchFirstStageDone_)
        {
            forward_ = !forward_;
            switchingDirections_ = false;
            setPosition_ = TwoJointArmProfiles::STOWED;
            position_ = TwoJointArmProfiles::STOWED;
            if (switchingToCubeIntake_)
            {
                wantedTheta = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][2];
                wantedPhi = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][3];
                switchingToCubeIntake_ = false;
                setPosition_ = TwoJointArmProfiles::CUBE_INTAKE;
                position_ = TwoJointArmProfiles::CUBE_INTAKE;
            }
            else
            {
                wantedTheta = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2];
                wantedPhi = TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3];
            }

            theta = getTheta();
            phi = getPhi();
        }

        // position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
        {
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD || abs(phi - wantedPhi) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
        {
            shoulderTraj_.generateTrajectory(theta, wantedTheta, 0);
            elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            state_ = HOLDING_POS;
        }
    }

    wantedPhiVel += wantedThetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    wantedPhiAcc += wantedThetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double thetaVolts = calcShoulderVolts(wantedThetaVel, wantedThetaAcc, wantedTheta, theta, phi, false);
    double phiVolts = calcElbowVolts(wantedPhiVel, wantedPhiAcc, wantedPhi, theta, phi, false);

    setShoulderVolts(thetaVolts);
    // shoulderMaster_.setVel(wantedThetaVel);
    // shoulderMaster_.setPos_(wantedTheta);
    setElbowVolts(phiVolts);
    // elbowMaster_.setVel(wantedPhiVel);
    // elbowMaster_.setPos_(wantedPhi);
}

void TwoJointArm::home()
{
    // claw_.setOpen(false);

    // double shoulderVel = getThetaVel(); // shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    // double elbowVel = getPhiVel();      // elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    double theta = getTheta();
    double phi = getPhi();

    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);
    if (!forward_)
    {
        xy.first *= -1;
    }

    // bool elbowRaiseNeeded = false;

    if (xy.first > 0)
    {
        cubeIntakeNeededDown_ = false;

        double aboveIntakeY = xy.second + TwoJointArmConstants::CONE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        double fromIntakeX = xy.first - TwoJointArmConstants::CONE_INTAKE_TO_SHOULDER_X;

        if (coneIntakeDown_)
        {
            coneIntakeNeededDown_ = true;
        }
        else if (sqrt(aboveIntakeY * aboveIntakeY + fromIntakeX * fromIntakeX) > TwoJointArmConstants::CONE_INTAKE_LENGTH + TwoJointArmConstants::CONE_INTAKE_COLLISION_BUFFER)
        {
            coneIntakeNeededDown_ = true;
        }
        else if (xy.first < TwoJointArmConstants::CONE_INTAKE_TO_SHOULDER_X - 0.15)
        {
            coneIntakeNeededDown_ = false;
        }
        else
        {
            // elbowRaiseNeeded = true;
            coneIntakeNeededDown_ = false;
        }
    }
    else
    {
        coneIntakeNeededDown_ = false;

        double aboveIntakeY = xy.second + TwoJointArmConstants::CUBE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        double fromIntakeX = xy.first + TwoJointArmConstants::CUBE_INTAKE_TO_SHOULDER_X;

        if (cubeIntakeDown_)
        {
            cubeIntakeNeededDown_ = true;
        }
        else if (sqrt(aboveIntakeY * aboveIntakeY + fromIntakeX * fromIntakeX) > TwoJointArmConstants::CUBE_INTAKE_LENGTH + TwoJointArmConstants::CUBE_INTAKE_COLLISION_BUFFER)
        {
            cubeIntakeNeededDown_ = true;
        }
        else if (xy.first > -TwoJointArmConstants::CUBE_INTAKE_TO_SHOULDER_X + 0.15)
        {
            cubeIntakeNeededDown_ = false;
        }
        else
        {
            // elbowRaiseNeeded = true;
            cubeIntakeNeededDown_ = false;
        }
    }

    // if(elbowRaiseNeeded)
    // {
    //     stop();
    //     return;
    // }

    std::tuple<double, double, double> shoulderProfile;
    std::tuple<double, double, double> elbowProfile;
    if (!homing_.first)
    {
        shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        // shoulderTraj_.generateTrajectory(theta, 0, shoulderVel); HOMING 0
        homing_.first = true;
        setBrakes(false, true);
        shoulderProfile = shoulderTraj_.getProfile();
    }
    else
    {
        shoulderProfile = shoulderTraj_.getProfile();
        // if(get<0>(shoulderProfile) == 0 && get<1>(shoulderProfile) == 0 && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3]) < 5)
        // {
        //     shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], shoulderVel);
        // }HOMING 0
    }

    if (!homing_.second && /*abs(theta) < 4 && HOMING 0*/ TwoJointArmConstants::MOUNTING_HEIGHT + TwoJointArmConstants::UPPER_ARM_LENGTH * cos(theta * M_PI / 180.0) > (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH - 0.1)) // TODO collisions suck ass
    {
        if (/*elbowRaiseNeeded*/ /*abs(phi - 100) > 10 HOMING 0*/ false) // HERE
        {
            elbowTraj_.generateTrajectory(phi, 100, 0);
            homing_.second = true;
            homingRaising_ = true;
            setBrakes(shoulderBrakeEngaged(), false);
            elbowProfile = elbowTraj_.getProfile();
        }
        else
        {
            elbowTraj_.generateTrajectory(phi, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
            homing_.second = true;
            homingRaising_ = false;
            setBrakes(shoulderBrakeEngaged(), false);
            elbowProfile = elbowTraj_.getProfile();
        }
    }
    else
    {
        elbowProfile = elbowTraj_.getProfile();
        // if(get<0>(elbowProfile) == 0 && get<1>(elbowProfile) == 0 && abs(theta) < 5 && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3]) > 10)
        // {
        //     shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], shoulderVel);
        // }HOMING 0
    }

    double wantedTheta = get<2>(shoulderProfile);
    double thetaVel = get<1>(shoulderProfile);
    double thetaAcc = get<0>(shoulderProfile);

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, wantedTheta, theta, phi, false);

    if (thetaVel == 0 && thetaAcc == 0)
    {
        thetaVolts = 0;
        setBrakes(true, elbowBrakeEngaged());
    }

    setShoulderVolts(thetaVolts);
    // shoulderMaster_.setVel(thetaVel);
    // shoulderMaster_.setPos_(wantedTheta);

    if (homing_.second)
    {
        double wantedPhi = get<2>(elbowProfile);
        double phiVel = get<1>(elbowProfile);
        double phiAcc = get<0>(elbowProfile);

        phiVel += thetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
        phiAcc += thetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

        double phiVolts = calcElbowVolts(phiVel, phiAcc, wantedPhi, theta, phi, false);
        // phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_;

        setElbowVolts(phiVolts);
        // elbowMaster_.setVel(phiVel);
        // elbowMaster_.setPos_(wantedPhi);

        if (/*phiVel == 0 && thetaVel == 0 && homingRaising_*/ false) // HERE
        {
            elbowTraj_.generateTrajectory(phi, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
            setBrakes(shoulderBrakeEngaged(), false);
            homingRaising_ = false;
            if (forward_)
            {
                coneIntakeNeededDown_ = true;
                cubeIntakeNeededDown_ = false;
            }
            else
            {
                coneIntakeNeededDown_ = false;
                cubeIntakeNeededDown_ = true;
            }
        }
        else if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
        {
            homing_ = {false, false};
            posUnknown_ = false;
            position_ = TwoJointArmProfiles::STOWED;
            if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
            {
                posUnknown_ = true;
                state_ = STOPPED;
            }
            else if (abs(theta - wantedTheta) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD || abs(phi - wantedPhi) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
            {
                shoulderTraj_.generateTrajectory(theta, wantedTheta, 0);
                elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
                state_ = FOLLOWING_JOINT_SPACE_PROFILE;
            }
            else
            {
                state_ = HOLDING_POS;
            }
        }
    }
}

void TwoJointArm::homeNew()
{
    // Homing:
    // put intakes on the side of the claw down
    // Bring the upper arm straight up (theta = 0) (if phi is basically extended, bring to 90)
    // If forearm is really close to 180, just set both to stow
    // Else bring forearm up to 90 so it doesn't obliterate itself or the intake, then bring both to stow ISSUE IF ON OTHER SIDE, DO FOREARM FIRST

    double theta = getTheta();
    double phi = getPhi();

    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);
    if (!forward_)
    {
        xy.first *= -1;
    }
    if (xy.first > 0)
    {
        cubeIntakeNeededDown_ = false;
        coneIntakeNeededDown_ = true;
    }
    else
    {
        cubeIntakeNeededDown_ = true;
        coneIntakeNeededDown_ = false;
    }

    std::tuple<double, double, double> shoulderProfile;
    std::tuple<double, double, double> elbowProfile;
    if (!homingFirstStage_.first) // Bring upper arm to 0
    {
        shoulderTraj_.generateTrajectory(theta, 0, 0);
        // shoulderProfile = shoulderTraj_.getProfile();
        homingFirstStage_.first = true;

        if (phi < 90)
        {
            homingFirstStage_.second = true;
            elbowTraj_.generateTrajectory(phi, 120, 0);
        }
        else if (phi > 270)
        {
            homingFirstStage_.second = true;
            elbowTraj_.generateTrajectory(phi, 240, 0);
        }

        // elbowProfile = elbowTraj_.getProfile();
    }
    shoulderProfile = shoulderTraj_.getProfile();

    if (homingFirstStage_.second)
    {
        elbowProfile = elbowTraj_.getProfile();
    }

    if (get<0>(shoulderProfile) == 0 && get<1>(shoulderProfile) == 0 && !homingSecondStage_.first) // Upper arm got to 0
    {
        if (!homingFirstStage_.second && (abs(abs(phi) - 180)) < 5) // If forearm is already in a good spot, skip intermediate steps and just home
        {
            homingFirstStage_.second = true;
            homingSecondStage_.second = true;
            homingSecondStage_.first = true;
            shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
            elbowTraj_.generateTrajectory(phi, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        }
        else if (!homingFirstStage_.second) // If forearm isn't in good spot and hasn't started homing, bring up to 90 to avoid intakes
        {
            homingFirstStage_.second = true;
            if (xy.first > 0)
            {
                if (forward_)
                {
                    elbowTraj_.generateTrajectory(phi, 120, 0);
                }
                else
                {
                    elbowTraj_.generateTrajectory(phi, 240, 0);
                }
            }
            else
            {
                if (forward_)
                {
                    elbowTraj_.generateTrajectory(phi, 240, 0);
                }
                else
                {
                    elbowTraj_.generateTrajectory(phi, 120, 0);
                }
            }
        }

        elbowProfile = elbowTraj_.getProfile();
        if (get<0>(elbowProfile) == 0 && get<1>(elbowProfile) == 0 && !homingSecondStage_.second) // Finished phi first stage but not second
        {
            homingSecondStage_.second = true;
            // homingSecondStage_.first = true;
            // shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
            elbowTraj_.generateTrajectory(phi, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3], 0);
        }
        else if (get<0>(elbowProfile) == 0 && get<1>(elbowProfile) == 0 && homingSecondStage_.second)
        {
            homingSecondStage_.first = true;
            shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2], 0);
        }
    }

    shoulderProfile = shoulderTraj_.getProfile();
    double wantedTheta = get<2>(shoulderProfile);
    double thetaVel = get<1>(shoulderProfile);
    double thetaAcc = get<0>(shoulderProfile);

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, wantedTheta, theta, phi, false);

    if (thetaVel == 0 && thetaAcc == 0)
    {
        thetaVolts = 0;
        setBrakes(true, elbowBrakeEngaged());
    }
    else
    {
        setBrakes(false, elbowBrakeEngaged());
    }
    setShoulderVolts(thetaVolts);

    if (homingFirstStage_.second)
    {
        elbowProfile = elbowTraj_.getProfile();
        double wantedPhi = get<2>(elbowProfile);
        double phiVel = get<1>(elbowProfile);
        double phiAcc = get<0>(elbowProfile);

        phiVel += thetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
        phiAcc += thetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

        double phiVolts = calcElbowVolts(phiVel, phiAcc, wantedPhi, theta, phi, false);

        if (phiVel == 0 && phiAcc == 0)
        {
            phiVolts = 0;
            setBrakes(shoulderBrakeEngaged(), true);
        }
        else
        {
            setBrakes(shoulderBrakeEngaged(), false);
        }
        setElbowVolts(phiVolts);

        if (homingSecondStage_.second && homingSecondStage_.first)
        {
            if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
            {
                homingFirstStage_ = {false, false};
                homingSecondStage_ = {false, false};
                posUnknown_ = false;
                position_ = TwoJointArmProfiles::STOWED;
                if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
                {
                    posUnknown_ = true;
                    state_ = STOPPED;
                }
                else if (abs(theta - wantedTheta) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD || abs(phi - wantedPhi) > TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
                {
                    shoulderTraj_.generateTrajectory(theta, wantedTheta, 0);
                    elbowTraj_.generateTrajectory(phi, wantedPhi, 0);
                    state_ = FOLLOWING_JOINT_SPACE_PROFILE;
                }
                else
                {
                    state_ = HOLDING_POS;
                }
            }
        }
    }
}

void TwoJointArm::toggleForward()
{
    if (state_ == HOLDING_POS && position_ == TwoJointArmProfiles::STOWED /* && !switchingDirections_*/)
    {
        // switchDirections();
        forward_ = !forward_;
    }
}

// void TwoJointArm::toggleForwardCubeIntake()
// {
//     if (state_ == HOLDING_POS && (position_ == TwoJointArmProfiles::STOWED || position_ == TwoJointArmProfiles::CUBE_INTAKE) && !switchingDirections_)
//     {
//         switchDirectionsCubeIntake();
//     }
// }

void TwoJointArm::toggleForwardExtendedToCubeIntake()
{
    if (position_ == TwoJointArmProfiles::STOWED)
    {
        // toggleForwardCubeIntake();
        toggleForward();
        setPosTo(TwoJointArmProfiles::CUBE_INTAKE); // NEUTRAL STOW
        return;
    }

    // if (state_ == HOLDING_POS && !posUnknown_ && position_ != TwoJointArmProfiles::CONE_INTAKE && position_ != TwoJointArmProfiles::CUBE_INTAKE && position_ != TwoJointArmProfiles::SPECIAL && position_ != TwoJointArmProfiles::GROUND && !switchingDirections_ && forward_)
    // {
    //     swingthroughExtendedToCubeIntake();
    // }

    if (state_ == HOLDING_POS && !posUnknown_ && (position_ == TwoJointArmProfiles::MID || position_ == TwoJointArmProfiles::HIGH || position_ == TwoJointArmProfiles::CUBE_MID || position_ == TwoJointArmProfiles::CUBE_HIGH) && forward_)
    {
        swingthroughExtendedToCubeIntake();
    }
}

void TwoJointArm::manualControl(double thetaVel, double phiVel, bool gravity)
{
    state_ = MANUAL;
    if (eStopped_)
    {
        return;
    }

    double theta = getTheta();
    double phi = getPhi();
    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);
    if (!forward_)
    {
        xy.first *= -1;
    }

    if (xy.first > TwoJointArmConstants::CONE_INTAKE_TO_SHOULDER_X - 0.15)
    {
        double aboveIntakeY = xy.second + TwoJointArmConstants::CONE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        double fromIntakeX = xy.first - TwoJointArmConstants::CONE_INTAKE_TO_SHOULDER_X;
        if (sqrt(aboveIntakeY * aboveIntakeY + fromIntakeX * fromIntakeX) < TwoJointArmConstants::CONE_INTAKE_LENGTH + TwoJointArmConstants::CONE_INTAKE_COLLISION_BUFFER)
        {
            coneIntakeNeededDown_ = true;
        }
        else
        {
            coneIntakeNeededDown_ = false;
        }
    }
    else
    {
        coneIntakeNeededDown_ = false;
    }

    if (xy.first < -TwoJointArmConstants::CUBE_INTAKE_TO_SHOULDER_X + 0.15)
    {
        double aboveIntakeY = xy.second + TwoJointArmConstants::CUBE_INTAKE_PIVOT_TO_SHOULDER_HEGHT;
        double fromIntakeX = xy.first + TwoJointArmConstants::CUBE_INTAKE_TO_SHOULDER_X;
        if (sqrt(aboveIntakeY * aboveIntakeY + fromIntakeX * fromIntakeX) < TwoJointArmConstants::CUBE_INTAKE_LENGTH + TwoJointArmConstants::CUBE_INTAKE_COLLISION_BUFFER)
        {
            cubeIntakeNeededDown_ = true;
        }
        else
        {
            cubeIntakeNeededDown_ = false;
        }
    }
    else
    {
        cubeIntakeNeededDown_ = false;
    }

    /*phiVel += thetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double thetaVolts = calcShoulderVolts(thetaVel, 0, getTheta(), getTheta(), getPhi(), false);
    double phiVolts = calcElbowVolts(phiVel, 0, getPhi(), getTheta(), getPhi(), false);

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
    elbowMaster_.SetVoltage(units::volt_t{phiVolts});*/

    // double volts = frc::SmartDashboard::GetNumber("Test Volts", 3);
    double thetaVolts = 3 * thetaVel;
    double phiVolts = 1.5 * phiVel;

    if (gravity)
    {
        double gravityTorque = calcElbowGravityTorque(getTheta(), getPhi(), false);
        phiVolts += gravityTorque * -0.0165 * 1.2; //-0.012 for pvc no claw
    }
    // frc::SmartDashboard::PutNumber("ET", gravityTorque);

    if (gravity)
    {
        double shoulderGravityTorque = calcShoulderGravityTorque(getTheta(), getPhi(), false);
        thetaVolts += shoulderGravityTorque * -0.005;
        // frc::SmartDashboard::PutNumber("ST", shoulderGravityTorque);
    }

    if (abs(thetaVel) < 0.05)
    {
        setBrakes(true, elbowBrakeEngaged());
        setShoulderVolts(0);

        // double gravityVolts = gravityTorque * -0.0165 * 1.0;
        // shoulderMaster_.SetVoltage(units::volt_t{gravityVolts});
    }
    else
    {
        bool moving = setShoulderVolts(thetaVolts);
        if(!moving)
        {
            setBrakes(true, elbowBrakeEngaged());
        }
        else
        {
            setBrakes(false, elbowBrakeEngaged());
        }
    }

    if (abs(phiVel) < 0.05)
    {
        if (gravity)
        {
            setBrakes(shoulderBrakeEngaged(), true);
        }
        else
        {
            setBrakes(shoulderBrakeEngaged(), false);
        }
        setElbowVolts(0);
    }
    else
    {
        bool moving = setElbowVolts(phiVolts);
        if(!moving)
        {
            setBrakes(shoulderBrakeEngaged(), true);
        }
        else
        {
            setBrakes(shoulderBrakeEngaged(), false);
        }
    }
    // New lenghts with gravity of -0.0165 * 1.2
    // 0.6, 0, na
    // 0.9, 15, 0
    // 1, 21, 4
    // 1.5, 40, 11
    // 2, 66, 19
    // 2.5, 86, 26
    // 3, 116, 32.5
    // 3.5, 131, 40
    // 4, 154, 46
    // 4.5, 53
    // 5, 59
    // 5.5, 66
    // 6, 73.5
}

double TwoJointArm::calcShoulderVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, bool hasCone)
{
    double gravityTorque = calcShoulderGravityTorque(theta, phi, hasCone);
    double rotInertia = calcShoulderRotInertia(phi, hasCone);
    double accTorque = (wantedAcc * M_PI / 180) * rotInertia;

    // torque *= TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO * 0.5;

    // double torqueVolts = (torque / abs(torque) * (abs(torque) - 0.948) / 0.3055;
    // double torqueVolts = torque * (calcR(prevShoulderVolts_) / calcKT(prevShoulderVolts_));
    // double torqueVolts = torque * (0.04669261 / 0.01824903);
    double gravityTorqueVolts = gravityTorque * -0.005;
    // TODO see which is best
    // COULDO torque with non-linear relationship
    double accTorqueVolts = accTorque * 0;

    // peenut

    double shoulderVel = getThetaVel();
    double thetaPID = (wantedPos - theta) * TwoJointArmConstants::skP_ + (wantedVel - shoulderVel) * TwoJointArmConstants::skD_;
    if (wantedVel == 0 && wantedAcc == 0)
    {
        thetaPID *= 0.4;
    }

    double velVolts;
    if (wantedVel == 0)
    {
        velVolts = 0;
    }
    else
    {
        velVolts = (abs(wantedVel) - TwoJointArmConstants::SHOULDER_KVI) / TwoJointArmConstants::SHOULDER_KV; // If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

        if (wantedVel < 0)
        {
            velVolts *= -1;
        }
    }

    // prevShoulderVolts_ = torqueVolts + velVolts;
    return gravityTorqueVolts + accTorqueVolts + velVolts + thetaPID;

    // // HERE
    // //double velVolts = (wantedVel) / (-47.1);
    // double velVolts = (abs(wantedVel) + 39.709) / 98.4508;
    // velVolts *= abs(wantedVel) / wantedVel;
    // //y=98.4508x-39.709

    // double gravityTorque = calcShoulderGravityTorque(theta, phi, false);
    // //velVolts += gravityTorque * (2.8 / 1.7);

    // //thetaVolts += (wantedTheta - theta) * skP_ + (wantedThetaVel - shoulderVel) * skV_; HERE
    // double thetaPID;
    // double shoulderVel = getThetaVel();
    // if(wantedVel == 0 && wantedAcc == 0)
    // {
    //     thetaPID = (wantedPos - theta) * -skP_ * 0.4 + (wantedVel - shoulderVel) * -skV_ * 0.4;
    // }
    // else
    // {
    //     thetaPID = (wantedPos - theta) * -skP_ + (wantedVel - shoulderVel) * -skV_;
    // }
    // //velVolts += thetaPID;

    // return velVolts;
}
double TwoJointArm::calcElbowVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, double hasCone)
{
    double gravityTorque = calcElbowGravityTorque(theta, phi, hasCone);
    double rotInertia = calcElbowRotInertia(hasCone);
    double accTorque = (wantedAcc * M_PI / 180) * rotInertia;

    // torque *= TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * 0.5;

    // double torqueVolts = (torque / abs(torque) * (abs(torque) - 0.948) / 0.3055;
    // double torqueVolts = torque * (calcR(prevElbowVolts_) / calcKT(prevElbowVolts_));
    // double torqueVolts = torque * (0.04669261 / 0.01824903);
    double gravityTorqueVolts = gravityTorque * -0.0165 * 1.2;
    // TODO see which is best
    // COULDO torque with non-linear relationship
    double accTorqueVolts = accTorque * 0;

    double elbowVel = getPhiVel();
    double phiPID = (wantedPos - phi) * TwoJointArmConstants::ekP_ + (wantedVel - elbowVel) * TwoJointArmConstants::ekD_;
    if (wantedVel == 0 && wantedAcc == 0)
    {
        phiPID *= 0.4;
    }

    double velVolts;
    if (wantedVel == 0)
    {
        velVolts = 0;
    }
    else
    {
        velVolts = (abs(wantedVel) - TwoJointArmConstants::ELBOW_KVI) / TwoJointArmConstants::ELBOW_KV; // If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

        if (wantedVel < 0)
        {
            velVolts *= -1;
        }
    }

    // prevElbowVolts_ = torqueVolts + velVolts;
    // return velVolts;
    return gravityTorqueVolts + accTorqueVolts + velVolts + phiPID;

    // // HERE
    // //double velVolts = (wantedVel) / (-100);
    // double velVolts = (abs(wantedVel) + 67.2685) / 183.796;
    // velVolts *= -abs(wantedVel) / wantedVel;
    // //y=183.796x-67.2685
    // // double accVolts = (abs(acc) / acc) * -0.5;
    // // double accVolts = (1.0 / 4.0) * 0.762 * 0.762 * 0.114 * (1.0 / 12.0) * -acc;

    // double gravityTorque = calcElbowGravityTorque(theta, phi, false);
    // //velVolts += gravityTorque * 3.24; // 1.0 / 2.6

    // //phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_; HERE
    // double phiPID;
    // double elbowVel = getPhiVel();
    // if(wantedVel == 0 && wantedAcc == 0)
    // {
    //     phiPID = (wantedPos - phi) * -ekP_ * 0.4 + (wantedVel - elbowVel) * -ekV_ * 0.4;
    // }
    // else
    // {
    //     phiPID = (wantedPos - phi) * -ekP_ + (wantedVel - elbowVel) * -ekV_;
    // }
    // //velVolts += phiPID;

    // return velVolts;
}

double TwoJointArm::calcShoulderRotInertia(double phi, bool hasCone)
{
    double shoulderToForearmCom = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_COM_DIST - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_COM_DIST * cos((180 - phi) * (M_PI / 180.0)));
    double noConeI = TwoJointArmConstants::SHOULDER_I + (TwoJointArmConstants::FOREARM_I * shoulderToForearmCom * shoulderToForearmCom);

    if (hasCone)
    {
        double shoulderToConeDist = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * cos((180 - phi) * (M_PI / 180.0)));
        return noConeI + GeneralConstants::CONE_M * shoulderToConeDist * shoulderToConeDist;
    }
    else
    {
        return noConeI;
    }
}
double TwoJointArm::calcElbowRotInertia(bool hasCone)
{
    if (hasCone)
    {
        return TwoJointArmConstants::ELBOW_I + ((TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * GeneralConstants::CONE_M);
    }
    else
    {
        return TwoJointArmConstants::ELBOW_I;
    }
}

double TwoJointArm::calcShoulderGravityTorque(double theta, double phi, bool hasCone)
{
    double upperArmTorque = TwoJointArmConstants::UPPER_ARM_COM_DIST * TwoJointArmConstants::UPPER_ARM_M * GeneralConstants::g * sin(theta * M_PI / 180.0);

    // COULDO move these calcs into ArmKinematics
    double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-theta) * (M_PI / 180.0));
    double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-theta) * (M_PI / 180.0));

    double secondJointAng_baseCords = theta + phi;

    double forearmCOMX = upperArmX - TwoJointArmConstants::FOREARM_COM_DIST * sin((-secondJointAng_baseCords) * (M_PI / 180.0));
    double forearmCOMY = upperArmY + TwoJointArmConstants::FOREARM_COM_DIST * cos((-secondJointAng_baseCords) * (M_PI / 180.0));

    double forearmTorque = sqrt(forearmCOMX * forearmCOMX + forearmCOMY * forearmCOMY) * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((M_PI / 2) - atan2(forearmCOMY, forearmCOMX));

    if (hasCone)
    {
        std::pair<double, double> xy = armKinematics_.angToXY(theta, phi);
        double coneTorque = sqrt(xy.first * xy.first + xy.second * xy.second) * GeneralConstants::CONE_M * GeneralConstants::g * sin((M_PI / 2) - atan2(xy.second, xy.first));
        return upperArmTorque + forearmTorque + coneTorque;
    }
    else
    {
        return upperArmTorque + forearmTorque;
    }
}
double TwoJointArm::calcElbowGravityTorque(double theta, double phi, bool hasCone)
{
    double forearmTorque = TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((theta + phi) * (M_PI / 180.0));

    if (hasCone)
    {
        return forearmTorque + (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH) * GeneralConstants::CONE_M * GeneralConstants::g * sin((theta + phi) * (M_PI / 180.0));
    }
    else
    {
        return forearmTorque;
    }
}

double TwoJointArm::calcKT(double volts)
{
    return 0.000646581 * volts * volts - 0.0147623 * volts + 0.102916;
}

double TwoJointArm::calcR(double volts)
{
    return 0.000758486 * volts * volts - 0.0187702 * volts + 0.163911;
}

bool TwoJointArm::setShoulderVolts(double volts)
{
    // frc::SmartDashboard::PutNumber("SC", shoulderMaster_.GetSupplyCurrent());
    double theta = getTheta();
    double phi = getPhi();
    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);
    if (shoulderMaster_.GetSupplyCurrent() > TwoJointArmConstants::STALL_SAFETY)
    {
        frc::SmartDashboard::PutNumber("SSC", shoulderMaster_.GetSupplyCurrent());
        shoulderMaster_.SetVoltage(units::volt_t(0));
        elbowMaster_.SetVoltage(units::volt_t(0));
        state_ = STOPPED;
        eStopped_ = true;
    }
    else if (theta > TwoJointArmConstants::SHOULDER_MAX_ANG && volts > 0)
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (theta < TwoJointArmConstants::SHOULDER_MIN_ANG && volts < 0)
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT /*+ 0.0254 * 2*/  - 0.0889 && theta > 0 && volts > 0) // Less than 6 inches from ground is * 2
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT /*+ 0.0254 * 2*/  - 0.0889 && theta < 0 && volts < 0) // Less than 6 inches from ground
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && abs(xy.first) < (SwerveConstants::WIDTH / 2.0) + 0.05 && theta > 0 && volts > 0) // Running into bellypan
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && abs(xy.first) < (SwerveConstants::WIDTH / 2.0) + 0.05 && theta < 0 && volts < 0) // Running into bellypan
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && xy.first > SwerveConstants::WIDTH / 2.0 && xy.first < (SwerveConstants::WIDTH / 2.0) + 0.0254 * 4 && volts > 0) // Running into side of bumper
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && xy.first < -SwerveConstants::WIDTH / 2.0 && xy.first > -((SwerveConstants::WIDTH / 2.0) + 0.0254 * 4) && volts < 0) // Running into side of bumper
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    }
    else if(xy.first > TwoJointArmConstants::MAX_X_EXENTIONS && volts > 0)
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    } 
    else if(xy.first < -TwoJointArmConstants::MAX_X_EXENTIONS && volts < 0)
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
    } 
    else
    {
        if (forward_)
        {
            shoulderMaster_.SetVoltage(units::volt_t{volts});
            return true;
        }
        else
        {
            shoulderMaster_.SetVoltage(units::volt_t{-volts});
            return true;
        }
    }

    return false;
}

bool TwoJointArm::setElbowVolts(double volts)
{
    // frc::SmartDashboard::PutNumber("EC", elbowMaster_.GetSupplyCurrent());
    double theta = getTheta();
    double phi = getPhi();
    std::pair<double, double> xy = ArmKinematics::angToXY(theta, phi);
    if (elbowMaster_.GetSupplyCurrent() > TwoJointArmConstants::STALL_SAFETY)
    {
        shoulderMaster_.SetVoltage(units::volt_t(0));
        elbowMaster_.SetVoltage(units::volt_t(0));
        state_ = STOPPED;
        eStopped_ = true;
    }
    else if (phi < TwoJointArmConstants::ELBOW_MIN_ANG && volts < 0)
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (phi > TwoJointArmConstants::ELBOW_MAX_ANG && volts > 0)
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT /*+ 0.0254 * 2*/  - 0.0889 && phi + theta > 0 && phi + theta < 180 && volts > 0) // Less than 6 inches above the ground
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT /*+ 0.0254 * 2*/  - 0.0889 && phi + theta > 180 && volts < 0) // Less than 6 inches above the ground
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && abs(xy.first) < (SwerveConstants::WIDTH / 2.0) + 0.05 && phi + theta > 0 && phi + theta < 180 && volts > 0) // Running into bellypan
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && abs(xy.first) < (SwerveConstants::WIDTH / 2.0) + 0.05 && phi + theta > 180 && volts < 0) // Running into bellypan
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && xy.first > SwerveConstants::WIDTH / 2.0 && xy.first < (SwerveConstants::WIDTH / 2.0) + 0.0254 * 4 && volts > 0) // Running into side of bumper
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if (xy.second < -TwoJointArmConstants::MOUNTING_HEIGHT + 0.0254 * 3 && xy.first < -SwerveConstants::WIDTH / 2.0 && xy.first > -((SwerveConstants::WIDTH / 2.0) + 0.0254 * 4) && volts < 0) // Running into side of bumper
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    }
    else if(xy.first > TwoJointArmConstants::MAX_X_EXENTIONS && volts > 0)
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    } 
    else if(xy.first < -TwoJointArmConstants::MAX_X_EXENTIONS && volts < 0) //55
    {
        elbowMaster_.SetVoltage(units::volt_t(0));
    } 
    else
    {
        if (forward_)
        {
            elbowMaster_.SetVoltage(units::volt_t{volts});
            return true;
        }
        else
        {
            elbowMaster_.SetVoltage(units::volt_t{-volts});
            return true;
        }
    }

    return false;
}

double TwoJointArm::getTheta()
{
    // return shoulderMaster_.GetSelectedSensorPosition(); //HERE
    //  double theta = shoulderMaster_.GetSelectedSensorPosition() / 2048 * 360 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    //  double theta = shoulderEncoder_.GetAbsolutePosition();
    double theta = -(shoulderEncoder_.GetAbsolutePosition() * 360.0) + TwoJointArmConstants::SHOULDER_ENCODER_OFFSET;

    // frc::SmartDashboard::PutNumber("GetSC", shoulderEncoder_.GetSourceChannel());
    // frc::SmartDashboard::PutNumber("GetAP", shoulderEncoder_.GetAbsolutePosition());
    // frc::SmartDashboard::PutNumber("Get", shoulderEncoder_.Get().value());
    // frc::SmartDashboard::PutNumber("GetDist", shoulderEncoder_.GetDistance());
    // frc::SmartDashboard::PutBoolean("Alive", shoulderEncoder_.IsConnected());
    Helpers::normalizeAngle(theta);
    return (forward_) ? theta : -theta;
}

double TwoJointArm::getPhi()
{
    // return elbowMaster_.GetSelectedSensorPosition(); //HERE
    if (forward_)
    {
        return elbowMaster_.GetSelectedSensorPosition() / 2048 * 360 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO - (getTheta() * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO);
    }
    else
    {
        return (360 - elbowMaster_.GetSelectedSensorPosition() / 2048 * 360 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO) - (getTheta() * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO);
    }
}

double TwoJointArm::getThetaVel()
{
    double vel = shoulderMaster_.GetSelectedSensorVelocity() / 2048 * 10 * 360 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    return (forward_) ? vel : -vel;
}

double TwoJointArm::getPhiVel()
{
    double vel = elbowMaster_.GetSelectedSensorVelocity() / 2048 * 10 * 360 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO - (getThetaVel() * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO);
    return (forward_) ? vel : -vel;
}

bool TwoJointArm::posUnknown()
{
    return posUnknown_;
}
bool TwoJointArm::shoulderBrakeEngaged()
{
    return shoulderBrakeEngaged_;
}
bool TwoJointArm::elbowBrakeEngaged()
{
    return elbowBrakeEngaged_;
}
bool TwoJointArm::isForward()
{
    return forward_;
}

// bool TwoJointArm::intaking()
// {
//     return intaking_;
// }

void TwoJointArm::checkPos()
{
    double theta = getTheta();
    double phi = getPhi();

    if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::STOWED_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::STOWED;
        state_ = HOLDING_POS;
    }
    else if (!forward_ && abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::CUBE_INTAKE;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::MID_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::MID;
        state_ = HOLDING_POS;
    }
    // else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::SPECIAL_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::SPECIAL_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    // {
    //     posUnknown_ = false;
    //     position_ = TwoJointArmProfiles::MID;
    //     state_ = HOLDING_POS;
    // } MID THING
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::HIGH_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::HIGH_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::HIGH;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_MID_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_MID_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::CUBE_MID;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_HIGH_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CUBE_HIGH_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::CUBE_HIGH;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::GROUND_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::GROUND_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::GROUND;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::RAMMING_PLAYER_STATION_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::RAMMING_PLAYER_STATION;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::AUTO_STOW_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::AUTO_STOW;
        state_ = HOLDING_POS;
    }
    // else if (forward_ && abs(theta - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][2]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD && abs(phi - TwoJointArmConstants::ARM_POSITIONS[TwoJointArmConstants::CONE_INTAKE_NUM][3]) < TwoJointArmConstants::ANGLE_POS_KNOWN_THRESHOLD)
    // {
    //     posUnknown_ = false;
    //     position_ = TwoJointArmProfiles::CONE_INTAKE;
    //     state_ = HOLDING_POS;
    // }
    else
    {
        posUnknown_ = true;
        state_ = STOPPED;
    }
}

std::string TwoJointArm::getStateString()
{
    switch (state_)
    {
    case HOLDING_POS:
    {
        return "Holding Pos";
        break;
    }
    case FOLLOWING_TASK_SPACE_PROFILE:
    {
        return "Following Task Space";
        break;
    }
    case FOLLOWING_JOINT_SPACE_PROFILE:
    {
        return "Following Joint Space";
        break;
    }
    case HOMING:
    {
        return "Homing";
        break;
    }
    case STOPPED:
    {
        return "Stopped";
        break;
    }
    case MANUAL:
    {
        return "Manual";
        break;
    }
    default:
    {
        return "UNKNOWN";
        break;
    }
    }
}

std::string TwoJointArm::getPosString()
{
    switch (position_)
    {
    case TwoJointArmProfiles::STOWED:
    {
        return "Stowed";
    }
    case TwoJointArmProfiles::CUBE_INTAKE:
    {
        return "Cube Intake";
    }
    case TwoJointArmProfiles::MID:
    {
        return "Mid";
    }
    case TwoJointArmProfiles::SPECIAL:
    {
        return "Special";
    }
    case TwoJointArmProfiles::HIGH:
    {
        return "High";
    }
    case TwoJointArmProfiles::CUBE_MID:
    {
        return "Cube Mid";
    }
    case TwoJointArmProfiles::CUBE_HIGH:
    {
        return "Cube High";
    }
    case TwoJointArmProfiles::GROUND:
    {
        return "Ground";
    }
    case TwoJointArmProfiles::RAMMING_PLAYER_STATION:
    {
        return "Player Station";
    }
    case TwoJointArmProfiles::AUTO_STOW:
    {
        return "Auto Stow";
    }
    // case TwoJointArmProfiles::CONE_INTAKE:
    // {
    //     return "Cone Intake";
    // }
    default:
    {
        return "UNKNOWN";
    }
    }
}

std::string TwoJointArm::getSetPosString()
{
    switch (setPosition_)
    {
    case TwoJointArmProfiles::STOWED:
    {
        return "Stowed";
    }
    case TwoJointArmProfiles::CUBE_INTAKE:
    {
        return "Cube Intake";
    }
    case TwoJointArmProfiles::MID:
    {
        return "Mid";
    }
    case TwoJointArmProfiles::SPECIAL:
    {
        return "Special";
    }
    case TwoJointArmProfiles::HIGH:
    {
        return "High";
    }
    case TwoJointArmProfiles::CUBE_MID:
    {
        return "Cube Mid";
    }
    case TwoJointArmProfiles::CUBE_HIGH:
    {
        return "Cube High";
    }
    case TwoJointArmProfiles::GROUND:
    {
        return "Ground";
    }
    case TwoJointArmProfiles::RAMMING_PLAYER_STATION:
    {
        return "Player Station";
    }
    case TwoJointArmProfiles::AUTO_STOW:
    {
        return "Auto Stow";
    }
    // case TwoJointArmProfiles::CONE_INTAKE:
    // {
    //     return "Cone Intake";
    // }
    default:
    {
        return "UNKNOWN";
    }
    }
}

// void TwoJointArm::goToPos(double thetaPos, double phiPos)
// {
//     state_ = MANUAL;

//     setBrakes(false, false);

//     if (setThetaPos_ != thetaPos)
//     {
//         shoulderTraj_.generateTrajectory(getTheta(), thetaPos, 0);
//         setThetaPos_ = thetaPos;
//     }

//     if (setPhiPos_ != phiPos)
//     {
//         elbowTraj_.generateTrajectory(getPhi(), phiPos, 0);
//         setPhiPos_ = phiPos;
//     }

//     double theta = getTheta();
//     double phi = getPhi();

//     std::tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile();
//     double wantedTheta = get<2>(shoulderProfile);
//     double wantedThetaVel = get<1>(shoulderProfile);
//     double shoulderVolts = calcShoulderVolts(wantedThetaVel, get<0>(shoulderProfile), wantedTheta, theta, phi, false);
//     // shoulderMaster_.SetVoltage(units::volt_t{shoulderVolts});
//     // frc::SmartDashboard::PutNumber("ShVolts", shoulderVolts);
//     setShoulderVolts(shoulderVolts);

//     std::tuple<double, double, double> elbowProfile = elbowTraj_.getProfile();
//     double wantedPhi = get<2>(elbowProfile);
//     double wantedPhiVel = get<1>(elbowProfile) + wantedThetaVel;
//     double elbowVolts = calcElbowVolts(wantedPhiVel, get<0>(elbowProfile) + get<0>(shoulderProfile), wantedPhi, theta, phi, false);
//     // elbowMaster_.SetVoltage(units::volt_t{elbowVolts});
//     // frc::SmartDashboard::PutNumber("ElVolts", elbowVolts);
//     setElbowVolts(elbowVolts);
// }

// double TwoJointArm::getThetaVolts()
// {
//     return shoulderMaster_.getVolts();
// }

// double TwoJointArm::getPhiVolts()
// {
//     return elbowMaster_.getVolts();
// }

bool TwoJointArm::getClawOpen()
{
    return claw_.isOpen();
}

void TwoJointArm::setClaw(bool open)
{
    claw_.setOpen(open);
}

void TwoJointArm::setClawWheels(double speed)
{
    claw_.setWheelSpeed(speed);
}

void TwoJointArm::setEStopped(bool eStopped)
{
    eStopped_ = eStopped;
}

bool TwoJointArm::isEStopped()
{
    return eStopped_;
}

std::pair<bool, bool> TwoJointArm::intakesNeededDown()
{
    return {cubeIntakeNeededDown_, coneIntakeNeededDown_};
}

void TwoJointArm::updateIntakeStates(bool cubeIntakeDown, bool coneIntakeDown)
{
    cubeIntakeDown_ = cubeIntakeDown;
    coneIntakeDown_ = coneIntakeDown;
}

void TwoJointArm::setForward(bool forward)
{
    forward_ = forward;
}

double TwoJointArm::getClawWheelSpeed()
{
    return claw_.wheelSpeed();
}

bool TwoJointArm::clawOpen()
{
    return claw_.isOpen();
}
