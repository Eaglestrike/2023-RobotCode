#include "Helpers/TrajectoryCalc.h"

using namespace Poses;

TrajectoryCalc::TrajectoryCalc(TrajectoryCalc::TrajCalcParam param) :
    parameters_(param)
{
}

void TrajectoryCalc::generateTrajectory(double pos, double setPos, double vel)
{
    //timer_.Reset();
    //timer_.Start();
    startTime_ = timer_.GetFPGATimestamp().value();

    setPos_ = setPos;
    initPos_ = pos;
    initVel_ = vel;

    double error = setPos - pos;
    //prevAbsoluteError_ = error;

    if(error > 0)
    {
        direction_ = 1;
    }
    else if(error < 0)
    {
        direction_ = -1;
    }
    else
    {
        direction_ = 0;
    }

    int initVelDistDirection;
    if (vel > 0)
    {
        initVelDistDirection = 1;
    }
    else if (vel < 0)
    {
        initVelDistDirection = -1;
    }
    else
    {
        initVelDistDirection = 0;
    }
    
    double initVelTime = abs(vel / parameters_.maxA);
    double initVelDistance = (initVelTime * vel) / 2;

    if(error == 0 && vel == 0)
    {
        accelDist_ = 0;
        accelTime_ = 0;
        cruiseSpeed_ = 0;
        cruiseDist_ = 0;
        cruiseTime_ = 0;
        deccelDist_ = 0;
        deccelTime_ = 0;
    }
    else if(error == initVelDistance)
    {
        accelDist_ = 0;
        accelTime_ = 0;
        cruiseSpeed_ = vel;
        cruiseDist_ = 0;
        cruiseTime_ = 0;
        deccelDist_ = error;
        deccelTime_ = initVelTime;
    }
    else if((error == 0 && vel != 0) || (abs(initVelDistance) > abs(error) && direction_ == initVelDistDirection) )
    {
        //problem child
        double newError = error - initVelDistance;

        double distanceToAccel = newError / 2;

        if (newError > 0)
        {
            direction_ = 1;
        }
        else if (newError < 0)
        {
            direction_ = -1;
        }

        cruiseSpeed_ = sqrt(2 * parameters_.maxA * abs(distanceToAccel));

        //cruiseSpeed_ = clamp(cruiseSpeed_, 0.0, parameters_.maxV);
        if (cruiseSpeed_ > parameters_.maxV)
        {
            cruiseSpeed_ = parameters_.maxV;
        }
        cruiseSpeed_ *= direction_;

        accelDist_ = initVelDistance + distanceToAccel;
        accelTime_ = (cruiseSpeed_ - vel) / parameters_.maxA * direction_;

        deccelDist_ = distanceToAccel;
        deccelTime_ = cruiseSpeed_ / parameters_.maxA * direction_;

        cruiseDist_ = 0;
        cruiseTime_ = 0;
    }
    else
    {
        double distanceToAccel = abs(error) + abs(initVelDistance);
        distanceToAccel /= 2;

        cruiseSpeed_ = sqrt(2 * parameters_.maxA * distanceToAccel);

        //cruiseSpeed_ = clamp(cruiseSpeed_, 0.0, parameters_.maxV);
        if (cruiseSpeed_ > parameters_.maxV)
        {
            cruiseSpeed_ = parameters_.maxV;
        }
        cruiseSpeed_ *= direction_;

        accelDist_ = ((cruiseSpeed_ * cruiseSpeed_) - (vel * vel)) / (2 * parameters_.maxA * direction_);
        accelTime_ = (cruiseSpeed_ - vel) / parameters_.maxA * direction_;

        deccelDist_ = (cruiseSpeed_ * cruiseSpeed_) / (2 * parameters_.maxA * direction_);
        deccelTime_ = cruiseSpeed_ / parameters_.maxA * direction_;

        cruiseDist_ = error - accelDist_ - deccelDist_;
        if(cruiseSpeed_ != 0)
        {
            cruiseTime_ = cruiseDist_ / cruiseSpeed_;
        }
        else
        {
            cruiseTime_ = 0;
        }
        
    }

}

// void TrajectoryCalc::generateVelTrajectory(double setVel, double vel)
// {
//     startTime_ = timer_.GetFPGATimestamp().value();

//     double error = setVel - vel;
//     initVel_ = vel;
//     setVel_ = setVel;
    
//     if(abs(error) < 100)
//     {
//         accelTime_ = 0;
//         direction_ = 0;
//     }
//     else
//     {
//         accelTime_ = abs(error / parameters_.maxA);
//         if(error > 0)
//         {
//             direction_ = 1;
//         }
//         else
//         {
//             direction_ = -1;
//         }
//     }

// }

/// @brief Get the current ideal pose for the motion of the trajectory at the current time
/// @return a Pose1D struct, containing info on position, velocity, and acceleration
Pose1D TrajectoryCalc::getProfile()
{
    double time = timer_.GetFPGATimestamp().value() - startTime_;
    Pose1D pose;
    if (time <= accelTime_ && accelTime_ >= 0.02) // first segment of trapezoidal motion: initial acceleration
    {
        pose.acc = parameters_.maxA * direction_; // MAX ACCELERATION, because optimal use of acceleration
        pose.vel = pose.acc * time + initVel_; // v = at + v0, kinematics
        pose.pos = ((pose.vel + initVel_) / 2) * time + initPos_; // x = (vf+v0)/2 * t + x0, position is average velocity x time 
    }
    else if (time <= accelTime_ && accelTime_ < 0.02) // 1st frame starts at initial conditions
    {
        pose.acc = 0;
        pose.vel = initVel_;
        pose.pos = initPos_;
    }
    else if (time > accelTime_ && time <= cruiseTime_ + accelTime_) // 2nd segment of trapezoidal motion: cruising
    {
        pose.acc = 0; //Idealy maintaining max velocity/cruiseSpeed
        pose.vel = cruiseSpeed_;
        pose.pos = cruiseSpeed_ * (time - accelTime_) + accelDist_ + initPos_; //x = vt + x0, where x0 is the sum of acceleration distance and initial position
    }
    else if (time > cruiseTime_ + accelTime_ && time < deccelTime_ + accelTime_ + cruiseTime_) //3rd segment of trapezoidal motion: final deacceleration
    {
        pose.acc = parameters_.maxA * -direction_; //Deaccelerate with max acceleration
        pose.vel = cruiseSpeed_ + pose.acc * (time - accelTime_ - cruiseTime_); // v = at + v0, kinematics
        pose.pos = (cruiseSpeed_ + pose.vel) / 2 * (time - accelTime_ - cruiseTime_) + accelDist_ + cruiseDist_ + initPos_; // x = (vf+v0)/2 * t + x0, position is average velocity x time 
    }
    else if (time > accelTime_ + deccelTime_ + cruiseTime_) //Time after reaching destination
    {
        pose.acc = 0;
        pose.vel = 0;
        pose.pos = setPos_; //Reaches final location with 0 vel
    }
    else //Time is negative
    {
        pose.acc = 0; //Die
        pose.vel = 0; //Die
        pose.pos = 0; //Die
    }

    return pose;

}

// pair<double, double> TrajectoryCalc::getVelProfile()
// {
//     double time = timer_.GetFPGATimestamp().value() - startTime_;
//     double tA, tV;

//     if(time >= accelTime_ || accelTime_ == 0)
//     {
//         tA = 0;
//         tV = setVel_;
//     }
//     else if(time < accelTime_)
//     {
//         tA = parameters_.maxA * direction_;
//         tV = initVel_ + (parameters_.maxA * direction_ * time);
//     }

//     return pair<double, double> (tA, tV);

// }

/// @brief returns the controller output at this time
/// @param pos current position reading
/// @param vel current velocity reading
/// @return the power clamped by max voltage in GeneralConstants
double TrajectoryCalc::calcPower(double pos, double vel)
{
    Pose1D profile = getProfile();

    double error = profile.pos - pos; //position error

    //double absoluteError = (setPos_ - pos);
    //double deltaAbsoluteError = absoluteError - prevAbsoluteError_;
    //prevAbsoluteError_ = absoluteError;
    double velError = profile.vel - vel; //velocity error
    if(printError_)
    {
        frc::SmartDashboard::PutNumber("TMError", error);
        frc::SmartDashboard::PutNumber("TMVError", velError);
    }
    double kVVolts; //Calculates kV elements of controller
    if(profile.vel == 0)
    {
        kVVolts = 0;
    }
    else
    {
        kVVolts = (abs(profile.vel) - parameters_.kVI) * parameters_.kV;
        if(profile.vel < 0)
        {
            kVVolts *= -1;
        }
    }

    //Adds all terms in the controller
    double power = (parameters_.kP * error) + (parameters_.kD * velError) + kVVolts + (profile.acc * parameters_.kA);

    if(abs(power) > GeneralConstants::MAX_VOLTAGE)
    {
        std::cout << "trapezoidal motion gains are shitting themselves" << std::endl;
    }

    power = std::clamp(power, -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE);
    return power;
}

// double TrajectoryCalc::calcVelPower(double vel)
// {
//     pair<double, double> profile = getVelProfile();

//     double error = profile.second - vel;

//     if(printError_)
//     {
//         frc::SmartDashboard::PutNumber("TMVError", error);
//     }

//     double kVVolts = (abs(profile.second) - parameters_.kVI) * parameters_.kV;
//     if(profile.second == 0)
//     {
//         kVVolts = 0;
//     }
//     else if(profile.second < 0)
//     {
//         kVVolts *= -1;
//     }

//     double power = (parameters_.kD * error) + (profile.first * parameters_.kA) + kVVolts;

//     if(abs(power) > GeneralConstants::MAX_VOLTAGE)
//     {
//         cout << "trapezoidal motion gains are shitting themselves" << endl;
//     }

//     power = clamp(power, -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE);
//     return power;
// }

void TrajectoryCalc::setPrintError(bool printError)
{
    printError_ = printError;
}

void TrajectoryCalc::setKP(double kP)
{
    parameters_.kP = kP;
}

void TrajectoryCalc::setKD(double kD)
{
    parameters_.kD = kD;
}

void TrajectoryCalc::setKV(double kV)
{
    parameters_.kV = kV;
}

void TrajectoryCalc::setKA(double kA)
{
    parameters_.kA = kA;
}

void TrajectoryCalc::setKVI(double kVI)
{
    parameters_.kVI = kVI;
}