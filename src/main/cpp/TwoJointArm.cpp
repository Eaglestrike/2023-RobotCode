#include "TwoJointArm.h"

TwoJointArm::TwoJointArm() : shoulderMaster_(TwoJointArmConstants::SHOULDER_MASTER_ID), shoulderSlave_(TwoJointArmConstants::SHOULDER_SLAVE_ID),
                             elbowMaster_(TwoJointArmConstants::ELBOW_MASTER_ID), elbowSlave_(TwoJointArmConstants::ELBOW_SLAVE_ID),
                             shoulderBrake_(frc::PneumaticsModuleType::CTREPCM, TwoJointArmConstants::SHOULDER_BRAKE_ID), elbowBrake_(frc::PneumaticsModuleType::CTREPCM, TwoJointArmConstants::ELBOW_BRAKE_ID),
                             shoulderTraj_(TwoJointArmConstants::SHOULDER_ARM_MAX_VEL, TwoJointArmConstants::SHOULDER_ARM_MAX_ACC, 0, 0, 0, 0), elbowTraj_(TwoJointArmConstants::ELBOW_ARM_MAX_VEL, TwoJointArmConstants::ELBOW_ARM_MAX_ACC, 0, 0, 0, 0)
{
    shoulderMaster_.SetNeutralMode(NeutralMode::Brake);
    shoulderSlave_.SetNeutralMode(NeutralMode::Brake);
    shoulderSlave_.Follow(shoulderMaster_);

    elbowMaster_.SetNeutralMode(NeutralMode::Brake);
    elbowSlave_.SetNeutralMode(NeutralMode::Brake);
    elbowSlave_.Follow(elbowMaster_); //HERE

    claw_.setOpen(false);

    movementProfiles_.readProfiles();
    state_ = STOPPED;
    position_ = TwoJointArmProfiles::STOWED;
    setPosition_ = TwoJointArmProfiles::STOWED;
    key_ = {TwoJointArmProfiles::STOWED, TwoJointArmProfiles::STOWED};
    posUnknown_ = true;
    zeroArms();
    forward_ = true;
    homing_ = {false, false};
    switchingDirections_ = false;
    intaking_ = false;
    gettingCone_ = false;
    gotCone_ = false;
    clawTimerStarted_ = false;
    setBrakes(true, true);
    taskSpaceStartTime_ = 0;

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

void TwoJointArm::periodic()
{

    frc::SmartDashboard::PutBoolean("Intaking", intaking_);

    switch (state_)
    {
    case HOLDING_POS:
    {
        if(posUnknown_)
        {
            state_ = STOPPED;
        }
        stop();
        if (position_ != setPosition_)
        {
            setPosTo(setPosition_);
        }
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
        setBrakes(false, false);
        followJointSpaceProfile();
        break;
    }
    case HOMING:
    {
        // setBrakes(false);
        home();
        break;
    }
    case STOPPED:
    {
        stop();
        resetIntaking();
        setPosition_ = position_;
        posUnknown_ = true;
        checkPos();
        break;
    }
    case MANUAL:
    {
        // setBrakes(false, false);
        posUnknown_ = true;
        break;
    }
    }

    if (intaking_)
    {
        intake();
    }
}

void TwoJointArm::zeroArms()
{
    shoulderMaster_.SetSelectedSensorPosition(0);
    elbowMaster_.SetSelectedSensorPosition(0);

    // shoulderMaster_.setPos_(6);
    // elbowMaster_.setPos_(154);
}

void TwoJointArm::setPosTo(TwoJointArmProfiles::Positions setPosition)
{   
    if(posUnknown_)
    {
        checkPos();
    }

    if (state_ == STOPPED || (setPosition == TwoJointArmProfiles::INTAKE && position_ != TwoJointArmProfiles::STOWED))
    {
        state_ = HOMING;
    }

    if (state_ == HOMING)
    {
        setPosition_ = setPosition;
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
    
    key_ = {position_, setPosition};

    setPosition_ = setPosition;
    state_ = FOLLOWING_TASK_SPACE_PROFILE;
    taskSpaceStartTime_ = timer_.GetFPGATimestamp().value();
}

void TwoJointArm::setBrakes(bool shoulder, bool elbow)
{
    shoulderBrake_.Set(shoulder);
    elbowBrake_.Set(elbow);
    shoulderBrakeEngaged_ = shoulder;
    elbowBrakeEngaged_ = elbow;
}
void TwoJointArm::stop()
{
    if (state_ == FOLLOWING_JOINT_SPACE_PROFILE || state_ == FOLLOWING_TASK_SPACE_PROFILE || state_ == HOMING || state_ == MANUAL)
    {
        state_ = STOPPED;
    }
    shoulderMaster_.SetVoltage(units::volt_t{0});
    elbowMaster_.SetVoltage(units::volt_t{0});
    setBrakes(true, true);
    homing_ = {false, false};
    switchingDirections_ = false;
}

void TwoJointArm::resetIntaking()
{
    intaking_ = false;
    gettingCone_ = false;
    gotCone_ = false;
    clawTimerStarted_ = false;
}

void TwoJointArm::switchDirections()
{
    double theta = getTheta();
    double phi = getPhi();
    double wantedTheta = -TwoJointArmConstants::sTheta;
    double wantedPhi = 360 - TwoJointArmConstants::sPhi;
    double thetaVel = getThetaVel(); // shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double phiVel = getPhiVel();     // elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - thetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    shoulderTraj_.generateTrajectory(theta, wantedTheta, thetaVel);
    elbowTraj_.generateTrajectory(phi, wantedPhi, phiVel);
    state_ = FOLLOWING_JOINT_SPACE_PROFILE;

    switchingDirections_ = true;

    claw_.setOpen(false);
}

void TwoJointArm::intake()
{
    if (state_ == MANUAL)
    {
        intaking_ = false;
        gettingCone_ = false;
        gotCone_ = false;
        clawTimerStarted_ = false;
        return;
    }

    if(!intaking_)
    {
        intaking_ = true;
    }

    if (!gettingCone_)
    {
        if ((position_ != TwoJointArmProfiles::STOWED || state_ == STOPPED))
        {
            // Not above the intake, move there
            stop();
            intaking_ = true;
            gettingCone_ = false;
            gotCone_ = false;
            clawTimerStarted_ = false;
            setPosTo(TwoJointArmProfiles::STOWED);
        }
        else if (state_ == HOLDING_POS)
        {
            // Above the intake, start getting the cone
            setPosTo(TwoJointArmProfiles::INTAKE);
            gettingCone_ = true;
        }
        else
        {
            // Moving to above the intake, wait
            return;
        }
    }
    else
    {
        // getting cone
        if (state_ == HOLDING_POS && !gotCone_)
        {
            if (position_ == TwoJointArmProfiles::STOWED)
            {
                setPosTo(TwoJointArmProfiles::INTAKE);
                claw_.setWheelState(Claw::INTAKING);
                claw_.setOpen(true);
            }
            else if (position_ == TwoJointArmProfiles::INTAKE)
            {
                if (!clawTimerStarted_)
                {
                    clawTimer_.Reset();
                    clawTimer_.Start();
                    clawTimerStarted_ = true;
                }

                if (clawTimer_.Get().value() > 0.5)
                {
                    claw_.setOpen(false);
                    setPosTo(TwoJointArmProfiles::STOWED);
                    gotCone_ = true;
                }
            }
        }
        else if (state_ == HOLDING_POS && gotCone_)
        {
            //Done
            intaking_ = false;
            gettingCone_ = false;
            gotCone_ = false;
            clawTimerStarted_ = false;
        }
    }
}

void TwoJointArm::placeCone()
{
    if (state_ != HOLDING_POS && state_ != MANUAL && state_ != STOPPED)
    {
        return;
    } //Driver preference

    //if(position_ == TwoJointArmProfiles::HIGH || position_ == TwoJointArmProfiles::MID)
    //{
        claw_.setOpen(true);
    //}//Driver preference
}

void TwoJointArm::placeCube()
{
    claw_.setWheelState(Claw::OUTAKING);
    claw_.setOpen(true);
}

void TwoJointArm::followTaskSpaceProfile(double time)
{
    tuple<double, double, double> thetaProfile = movementProfiles_.getThetaProfile(key_, time);
    tuple<double, double, double> phiProfile = movementProfiles_.getPhiProfile(key_, time);

    double wantedTheta = get<0>(thetaProfile);
    double wantedPhi = get<0>(phiProfile);
    double wantedThetaVel = get<1>(thetaProfile);
    double wantedPhiVel = get<1>(phiProfile);
    double wantedThetaAcc = get<2>(thetaProfile);
    double wantedPhiAcc = get<2>(phiProfile);

    double theta = getTheta();
    double phi = getPhi();
    double shoulderVel = getThetaVel(); // shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = getPhiVel();      // elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    if (wantedThetaVel == 0 && wantedPhiVel == 0 && wantedThetaAcc == 0 && wantedPhiAcc == 0)
    {
        position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
        {
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
        {
            shoulderTraj_.generateTrajectory(theta, wantedTheta, shoulderVel);
            elbowTraj_.generateTrajectory(phi, wantedPhi, elbowVel);
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            state_ = HOLDING_POS;
        }
    }

    wantedPhiVel += wantedThetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    wantedPhiAcc += wantedThetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    frc::SmartDashboard::PutNumber("WTVel", wantedThetaVel);
    frc::SmartDashboard::PutNumber("WPVel", wantedPhiVel);
    frc::SmartDashboard::PutNumber("WTPos", wantedTheta);
    frc::SmartDashboard::PutNumber("WPPos", wantedPhi);
    double thetaVolts = calcShoulderVolts(wantedThetaVel, wantedThetaAcc, wantedTheta, theta, phi, false); // TODO check if has cone is viable or necesary
    double phiVolts = calcElbowVolts(wantedPhiVel, wantedPhiAcc, wantedPhi, theta, phi, false);

    if (!forward_)
    {
        thetaVolts *= -1;
        phiVolts *= -1;
    }

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
    // shoulderMaster_.setVel(wantedThetaVel);
    // shoulderMaster_.setPos_(wantedTheta);
    elbowMaster_.SetVoltage(units::volt_t{phiVolts});
    // elbowMaster_.setVel(wantedPhiVel);
    // elbowMaster_.setPos_(wantedPhi);
}

void TwoJointArm::followJointSpaceProfile()
{
    tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile();
    tuple<double, double, double> elbowProfile = elbowTraj_.getProfile();

    double wantedTheta = get<2>(shoulderProfile);
    double wantedPhi = get<2>(elbowProfile);
    double wantedThetaVel = get<1>(shoulderProfile);
    double wantedPhiVel = get<1>(elbowProfile);
    double wantedThetaAcc = get<0>(shoulderProfile);
    double wantedPhiAcc = get<0>(elbowProfile);

    double theta = getTheta();
    double phi = getPhi();
    double shoulderVel = getThetaVel(); // shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = getPhiVel();      // elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    if (wantedThetaVel == 0 && wantedPhiVel == 0 && wantedThetaAcc == 0 && wantedPhiAcc == 0)
    {
        if (switchingDirections_)
        {
            forward_ = !forward_;
            switchingDirections_ = false;
            wantedTheta = 40;
            wantedPhi = 130;
            theta = getTheta();
            phi = getPhi();
        }

        //position_ = setPosition_;
        if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
        {
            state_ = STOPPED;
        }
        else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
        {
            shoulderTraj_.generateTrajectory(theta, wantedTheta, shoulderVel);
            elbowTraj_.generateTrajectory(phi, wantedPhi, elbowVel);
            state_ = FOLLOWING_JOINT_SPACE_PROFILE;
        }
        else
        {
            state_ = HOLDING_POS;
        }
    }

    wantedPhiVel += wantedThetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    wantedPhiAcc += wantedThetaAcc * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double thetaVolts = calcShoulderVolts(wantedThetaVel, wantedThetaAcc, wantedTheta, theta, phi, false); // TODO check if has cone is viable or necesary
    double phiVolts = calcElbowVolts(wantedPhiVel, wantedPhiAcc, wantedPhi, theta, phi, false);

    if (!forward_)
    {
        thetaVolts *= -1;
        phiVolts *= -1;
    }

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
    // shoulderMaster_.setVel(wantedThetaVel);
    // shoulderMaster_.setPos_(wantedTheta);
    elbowMaster_.SetVoltage(units::volt_t{phiVolts});
//     elbowMaster_.setVel(wantedPhiVel);
//     elbowMaster_.setPos_(wantedPhi);
}

void TwoJointArm::home()
{
    claw_.setOpen(false);

    double shoulderVel = getThetaVel(); // shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;
    double elbowVel = getPhiVel();      // elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - shoulderVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;
    double theta = getTheta();
    double phi = getPhi();

    if (!homing_.first)
    {
        shoulderTraj_.generateTrajectory(theta, TwoJointArmConstants::sTheta, shoulderVel);
        homing_.first = true;
        setBrakes(false, true);
    }

    if (!homing_.second && TwoJointArmConstants::MOUNTING_HEIGHT + TwoJointArmConstants::UPPER_ARM_LENGTH * cos(theta * pi / 180.0) > (TwoJointArmConstants::FOREARM_LENGTH + TwoJointArmConstants::EE_LENGTH - 0.1)) // TODO collisions suck ass
    {
        elbowTraj_.generateTrajectory(phi, TwoJointArmConstants::sPhi, elbowVel);
        homing_.second = true;
        setBrakes(shoulderBrakeEngaged(), false);
    }

    tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile();
    tuple<double, double, double> elbowProfile = elbowTraj_.getProfile();

    double wantedTheta = get<2>(shoulderProfile);
    double thetaVel = get<1>(shoulderProfile);
    double thetaAcc = get<0>(shoulderProfile);

    double thetaVolts = calcShoulderVolts(thetaVel, thetaAcc, wantedTheta, theta, phi, false); // TODO check if has cone is viable or necesary

    if (thetaVel == 0 && thetaAcc == 0)
    {
        thetaVolts = 0;
        setBrakes(true, elbowBrakeEngaged());
    }

    if (!forward_)
    {
        thetaVolts *= -1;
    }

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
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
        //phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_;

        if (!forward_)
        {
            phiVolts *= -1;
        }
        elbowMaster_.SetVoltage(units::volt_t{phiVolts});
        // elbowMaster_.setVel(phiVel);
        // elbowMaster_.setPos_(wantedPhi);

        if (thetaVel == 0 && phiVel == 0 && thetaAcc == 0 && phiAcc == 0)
        {
            homing_ = {false, false};
            posUnknown_ = false;
            position_ = TwoJointArmProfiles::STOWED;
            if (abs(theta - wantedTheta) > 20 || abs(phi - wantedPhi) > 20) // TODO get values
            {
                state_ = STOPPED;
            }
            else if (abs(theta - wantedTheta) > 10 || abs(phi - wantedPhi) > 10)
            {
                shoulderTraj_.generateTrajectory(theta, wantedTheta, thetaVel);
                elbowTraj_.generateTrajectory(phi, wantedPhi, phiVel);
                state_ = FOLLOWING_JOINT_SPACE_PROFILE;
            }
            else
            {
                state_ = HOLDING_POS;
            }
        }
    }
}

void TwoJointArm::toggleForward()
{
    if (state_ == HOLDING_POS && position_ == TwoJointArmProfiles::STOWED)
    {
        switchDirections();
        // forward_ = !forward_;
    }
}

void TwoJointArm::manualControl(double thetaVel, double phiVel)
{
    state_ = MANUAL;

    /*phiVel += thetaVel * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double thetaVolts = calcShoulderVolts(thetaVel, 0, getTheta(), getTheta(), getPhi(), false); //TODO check has cone
    double phiVolts = calcElbowVolts(phiVel, 0, getPhi(), getTheta(), getPhi(), false);

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
    elbowMaster_.SetVoltage(units::volt_t{phiVolts});*/
    // HERE

    double volts = frc::SmartDashboard::GetNumber("Test Volts", 0);
    double thetaVolts = volts * thetaVel * 2 / TwoJointArmConstants::SHOULDER_ARM_MAX_VEL;
    double phiVolts = volts * phiVel * 2 / TwoJointArmConstants::ELBOW_ARM_MAX_VEL;
    if (abs(thetaVel * 2 / TwoJointArmConstants::SHOULDER_ARM_MAX_VEL) < 0.2)
    {
        setBrakes(true, elbowBrakeEngaged());
    }
    else
    {
        setBrakes(false, elbowBrakeEngaged());
    }

    if (abs(phiVel * 2 / TwoJointArmConstants::ELBOW_ARM_MAX_VEL) < 0.2)
    {

        setBrakes(shoulderBrakeEngaged(), true);
    }
    else
    {
        setBrakes(shoulderBrakeEngaged(), false);
    }

    // double elbowVel = 360 * 10 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * elbowMaster_.GetSelectedSensorVelocity() / 4096.0;

    double gravityTorque = calcElbowGravityTorque(getTheta(), getPhi(), false);
    //phiVolts += gravityTorque * 3.24;
    // cout << getPhi() << ", " << elbowVel_ << endl;
    // cout << getTheta() << ", " << shoulderVel_ << endl;
    //frc::SmartDashboard::PutNumber("Manual TVel", getThetaVel());
    //frc::SmartDashboard::PutNumber("Manual PVel", getPhiVel());
    //0.6, 20, 40
    //1, 58, 120
    //1.5, 110, 210
    //2, 160, 200
    //3, 245, 480
    //4, 360, 670

    double shoulderGravityTorque = calcShoulderGravityTorque(getTheta(), getPhi(), false);
    //thetaVolts += shoulderGravityTorque * (2.8 / 1.7);

    shoulderMaster_.SetVoltage(units::volt_t{thetaVolts});
    elbowMaster_.SetVoltage(units::volt_t{phiVolts});
}

double TwoJointArm::calcShoulderVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, bool hasCone)
{
    /*double gravityTorque = calcShoulderGravityTorque(theta, phi, hasCone);
    double rotInertia = calcShoulderRotInertia(phi, hasCone);
    double accTorque = acc * rotInertia;

    double torque = gravityTorque + accTorque;
    torque *= TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO * 0.5;
    // double torqueVolts = (torque / abs(torque) * (abs(torque) - 0.948) / 0.3055;
    // double torqueVolts = torque * (calcR(prevShoulderVolts_) / calcKT(prevShoulderVolts_));
    double torqueVolts = torque * (0.04669261 / 0.01824903);
    // TODO see which is best
    // COULDO torque with non-linear relationship

    // Shoulder bag
    // 1.25, 90 with elbow closed

    // Elbow bag, lifts
    // 0, 180-95 no this
    // 1, everything but at 10 degrees?
    //
    // peenut

    double velVolts = (vel)*TwoJointArmConstants::SHOULDER_KV + TwoJointArmConstants::SHOULDER_KVI; // If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

    prevShoulderVolts_ = torqueVolts + velVolts;
    return torqueVolts + velVolts;*/

    // HERE
    //double velVolts = (wantedVel) / (-47.1);
    double velVolts = (abs(wantedVel) + 39.709) / 98.4508;
    velVolts *= abs(wantedVel) / wantedVel;
    //y=98.4508x-39.709

    double gravityTorque = calcShoulderGravityTorque(theta, phi, false);
    //velVolts += gravityTorque * (2.8 / 1.7);

    //thetaVolts += (wantedTheta - theta) * skP_ + (wantedThetaVel - shoulderVel) * skV_; HERE
    double thetaPID;
    double shoulderVel = getThetaVel();
    if(wantedVel == 0 && wantedAcc == 0)
    {
        thetaPID = (wantedPos - theta) * -skP_ * 0.4 + (wantedVel - shoulderVel) * -skV_ * 0.4;
    }
    else
    {
        thetaPID = (wantedPos - theta) * -skP_ + (wantedVel - shoulderVel) * -skV_;
    }
    //velVolts += thetaPID;

    return velVolts; //HERE, negative
}
double TwoJointArm::calcElbowVolts(double wantedVel, double wantedAcc, double wantedPos, double theta, double phi, double hasCone)
{
    /*double gravityTorque = calcElbowGravityTorque(theta, phi, hasCone);
    double rotInertia = calcElbowRotInertia(hasCone);
    double accTorque = acc * rotInertia;

    double torque = gravityTorque + accTorque;
    torque *= TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO * 0.5;
    // double torqueVolts = (torque / abs(torque) * (abs(torque) - 0.948) / 0.3055;
    // double torqueVolts = torque * (calcR(prevElbowVolts_) / calcKT(prevElbowVolts_));
    double torqueVolts = torque * (0.04669261 / 0.01824903);
    // TODO see which is best
    // COULDO torque with non-linear relationship

    double velVolts = (vel)*TwoJointArmConstants::ELBOW_KV + TwoJointArmConstants::ELBOW_KVI; // If gotten directly, all good. If using motor curves remember to convert to radians/sec and use gear ratio

    prevElbowVolts_ = torqueVolts + velVolts;
    return torqueVolts + velVolts;*/

    // HERE
    //double velVolts = (wantedVel) / (-100);
    double velVolts = (abs(wantedVel) + 67.2685) / 183.796;
    velVolts *= -abs(wantedVel) / wantedVel;
    //y=183.796x-67.2685
    // double accVolts = (abs(acc) / acc) * -0.5;
    // double accVolts = (1.0 / 4.0) * 0.762 * 0.762 * 0.114 * (1.0 / 12.0) * -acc;

    double gravityTorque = calcElbowGravityTorque(theta, phi, false);
    //velVolts += gravityTorque * 3.24; // 1.0 / 2.6

    //phiVolts += (wantedPhi - phi) * ekP_ + (phiVel - elbowVel) * ekV_; HERE
    double phiPID;
    double elbowVel = getPhiVel();
    if(wantedVel == 0 && wantedAcc == 0)
    {
        phiPID = (wantedPos - phi) * -ekP_ * 0.4 + (wantedVel - elbowVel) * -ekV_ * 0.4;
    }
    else
    {
        phiPID = (wantedPos - phi) * -ekP_ + (wantedVel - elbowVel) * -ekV_;
    }
    //velVolts += phiPID;

    return velVolts;
}

double TwoJointArm::calcShoulderRotInertia(double phi, bool hasCone)
{
    double shoulderToForearmCom = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_COM_DIST - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_COM_DIST * cos((180 - phi) * (pi / 180.0)));
    double noConeI = TwoJointArmConstants::SHOULDER_I + (TwoJointArmConstants::FOREARM_I * shoulderToForearmCom * shoulderToForearmCom);

    if (hasCone)
    {
        double shoulderToConeDist = sqrt(TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::UPPER_ARM_LENGTH + TwoJointArmConstants::FOREARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH - 2 * TwoJointArmConstants::UPPER_ARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH * cos((180 - phi) * (pi / 180.0)));
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
        return TwoJointArmConstants::ELBOW_I + (TwoJointArmConstants::FOREARM_LENGTH * TwoJointArmConstants::FOREARM_LENGTH * GeneralConstants::CONE_M);
    }
    else
    {
        return TwoJointArmConstants::ELBOW_I;
    }
}

double TwoJointArm::calcShoulderGravityTorque(double theta, double phi, bool hasCone)
{
    double upperArmTorque = TwoJointArmConstants::UPPER_ARM_COM_DIST * TwoJointArmConstants::UPPER_ARM_M * GeneralConstants::g * sin(theta * pi / 180.0);

    // COULDO move these calcs into ArmKinematics
    double upperArmX = -TwoJointArmConstants::UPPER_ARM_LENGTH * sin((-theta) * (pi / 180.0));
    double upperArmY = TwoJointArmConstants::UPPER_ARM_LENGTH * cos((-theta) * (pi / 180.0));

    double secondJointAng_baseCords = theta + phi;

    double forearmCOMX = upperArmX - TwoJointArmConstants::FOREARM_COM_DIST * sin((-secondJointAng_baseCords) * (pi / 180.0));
    double forearmCOMY = upperArmY + TwoJointArmConstants::FOREARM_COM_DIST * cos((-secondJointAng_baseCords) * (pi / 180.0));

    double forearmTorque = sqrt(forearmCOMX * forearmCOMX + forearmCOMY * forearmCOMY) * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((pi / 2) - atan2(forearmCOMY, forearmCOMX));

    if (hasCone)
    {
        pair<double, double> xy = armKinematics_.angToXY(theta, phi);
        double coneTorque = sqrt(xy.first * xy.first + xy.second * xy.second) * GeneralConstants::CONE_M * GeneralConstants::g * sin((pi / 2) - atan2(xy.second, xy.first));
        return upperArmTorque + forearmTorque + coneTorque;
    }
    else
    {
        return upperArmTorque + forearmTorque;
    }
}
double TwoJointArm::calcElbowGravityTorque(double theta, double phi, bool hasCone)
{
    double forearmTorque = TwoJointArmConstants::FOREARM_COM_DIST * TwoJointArmConstants::FOREARM_M * GeneralConstants::g * sin((theta + phi) * (pi / 180.0));

    if (hasCone)
    {
        return forearmTorque + TwoJointArmConstants::FOREARM_LENGTH * GeneralConstants::CONE_M * GeneralConstants::g * sin((theta + phi) * (pi / 180.0));
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

double TwoJointArm::getTheta()
{
    return -shoulderMaster_.GetSelectedSensorPosition() * (360.0 / 4096.0); // HERE
    //return shoulderMaster_.GetSelectedSensorPosition(); // TODO with encoders, don't forget switching sides
}

double TwoJointArm::getPhi()
{
    return elbowMaster_.GetSelectedSensorPosition() * (360.0 / 4096.0) - getTheta(); // HERE
    //return elbowMaster_.GetSelectedSensorPosition(); // TODO with encoders, don't forget switching sides
}

double TwoJointArm::getThetaVel()
{
    // return ((forward_) ? 1 : -1) * shoulderMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_SHOULDER_RATIO;

    double dt = timer_.GetFPGATimestamp().value() - shoulderVelTime_;

    double shoulderVel = (getTheta() - prevShoulderPos_) / dt;
    if (shoulderVel != 0)
    {
        shoulderVel_ = shoulderVel;
        prevShoulderPos_ = getTheta();
        shoulderVelTime_ = timer_.GetFPGATimestamp().value();
    }
    frc::SmartDashboard::PutNumber("shoulder vel", shoulderVel_);
    return ((forward_) ? 1 : -1) * shoulderVel_; // HERE
}

double TwoJointArm::getPhiVel()
{
    // return ((forward_) ? 1 : -1) * elbowMaster_.GetSelectedSensorVelocity() * (10.0 / 2048.0) * 360.0 * TwoJointArmConstants::MOTOR_TO_ELBOW_RATIO - getThetaVel() * TwoJointArmConstants::SHOULDER_TO_ELBOW_RATIO;

    double dt = timer_.GetFPGATimestamp().value() - elbowVelTime_;
    double elbowVel = (getPhi() - prevElbowPos_) / dt;
    if (elbowVel != 0)
    {
        elbowVel_ = elbowVel;
        prevElbowPos_ = getPhi();
        elbowVelTime_ = timer_.GetFPGATimestamp().value();
    }
    frc::SmartDashboard::PutNumber("elbow vel", elbowVel_);
    return ((forward_) ? 1 : -1) * elbowVel_; // HERE
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

void TwoJointArm::checkPos()
{
    double theta = getTheta();
    double phi = getPhi();

    if (abs(theta - TwoJointArmConstants::sTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::sPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::STOWED;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::giTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::giPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::GROUND_INTAKE;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::psTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::psPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::PLAYER_STATION;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::mTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::mPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::MID;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::hTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::hPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::HIGH;
        state_ = HOLDING_POS;
    }
    else if (abs(theta - TwoJointArmConstants::iTheta) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD && abs(phi - TwoJointArmConstants::iPhi) < TwoJointArmConstants::ANGLE_ERROR_THRESHOLD)
    {
        posUnknown_ = false;
        position_ = TwoJointArmProfiles::INTAKE;
        state_ = HOLDING_POS;
    }
    else
    {
        posUnknown_ = true;
        state_ = STOPPED;
    }
}

string TwoJointArm::getStateString()
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

string TwoJointArm::getPosString()
{
    switch (position_)
    {
    case TwoJointArmProfiles::STOWED:
    {
        return "Stowed";
        break;
    }
    case TwoJointArmProfiles::GROUND_INTAKE:
    {
        return "Ground Intake";
        break;
    }
    case TwoJointArmProfiles::PLAYER_STATION:
    {
        return "Player Station";
        break;
    }
    case TwoJointArmProfiles::MID:
    {
        return "Mid";
        break;
    }
    case TwoJointArmProfiles::HIGH:
    {
        return "High";
        break;
    }
    case TwoJointArmProfiles::INTAKE:
    {
        return "Intake";
        break;
    }
    default:
    {
        return "UNKNOWN";
        break;
    }
    }
}

void TwoJointArm::goToPos(double thetaPos, double phiPos)
{
    if (setThetaPos_ != thetaPos)
    {
        shoulderTraj_.generateTrajectory(getTheta(), thetaPos, 0);
        setThetaPos_ = thetaPos;
    }

    if (setPhiPos_ != phiPos)
    {
        elbowTraj_.generateTrajectory(getPhi(), phiPos, 0);
        setPhiPos_ = phiPos;
    }

    double theta = getTheta();
    double phi = getPhi();

    tuple<double, double, double> shoulderProfile = shoulderTraj_.getProfile();
    double wantedTheta = get<2>(shoulderProfile);
    double wantedThetaVel = get<1>(shoulderProfile);
    double shoulderVolts = calcShoulderVolts(wantedThetaVel, get<0>(shoulderProfile), wantedTheta, theta, phi, false);
    shoulderMaster_.SetVoltage(units::volt_t{shoulderVolts});

    tuple<double, double, double> elbowProfile = elbowTraj_.getProfile();
    double wantedPhi = get<2>(elbowProfile);
    double wantedPhiVel = get<1>(elbowProfile);
    double elbowVolts = calcElbowVolts(wantedPhiVel, get<0>(elbowProfile) + get<0>(shoulderProfile), wantedPhi, theta, phi, false);
    elbowMaster_.SetVoltage(units::volt_t{elbowVolts});
}

// double TwoJointArm::getThetaVolts()
// {
//     return shoulderMaster_.getVolts();
// }

// double TwoJointArm::getPhiVolts()
// {
//     return elbowMaster_.getVolts();
// }