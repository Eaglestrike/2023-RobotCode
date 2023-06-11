#include "TrajectoryCalc.h"

TrajectoryCalc::TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA) : MAX_V(maxV), MAX_A(maxA), kP_(kP), kD_(kD), kV_(kV), kA_(kA), kVI_(0)
{
    
}

TrajectoryCalc::TrajectoryCalc(double maxV, double maxA, double kP, double kD, double kV, double kA, double kVI) : MAX_V(maxV), MAX_A(maxA), kP_(kP), kD_(kD), kV_(kV), kA_(kA), kVI_(kVI)
{

}


void TrajectoryCalc::setKP(double kP)
{
    kP_ = kP;
}

void TrajectoryCalc::setKD(double kD)
{
    kD_ = kD;
}

void TrajectoryCalc::setKV(double kV)
{
    kV_ = kV;
}

void TrajectoryCalc::setKA(double kA)
{
    kA_ = kA;
}

void TrajectoryCalc::setKVI(double kVI)
{
    kVI_ = kVI;
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
    
    double initVelTime = abs(vel / MAX_A);
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

        cruiseSpeed_ = sqrt(2 * MAX_A * abs(distanceToAccel));

        //cruiseSpeed_ = clamp(cruiseSpeed_, 0.0, MAX_V);
        if (cruiseSpeed_ > MAX_V)
        {
            cruiseSpeed_ = MAX_V;
        }
        cruiseSpeed_ *= direction_;

        accelDist_ = initVelDistance + distanceToAccel;
        accelTime_ = (cruiseSpeed_ - vel) / MAX_A * direction_;

        deccelDist_ = distanceToAccel;
        deccelTime_ = cruiseSpeed_ / MAX_A * direction_;

        cruiseDist_ = 0;
        cruiseTime_ = 0;
    }
    else
    {
        double distanceToAccel = abs(error) + abs(initVelDistance);
        distanceToAccel /= 2;

        cruiseSpeed_ = sqrt(2 * MAX_A * distanceToAccel);

        //cruiseSpeed_ = clamp(cruiseSpeed_, 0.0, MAX_V);
        if (cruiseSpeed_ > MAX_V)
        {
            cruiseSpeed_ = MAX_V;
        }
        cruiseSpeed_ *= direction_;

        accelDist_ = ((cruiseSpeed_ * cruiseSpeed_) - (vel * vel)) / (2 * MAX_A * direction_);
        accelTime_ = (cruiseSpeed_ - vel) / MAX_A * direction_;

        deccelDist_ = (cruiseSpeed_ * cruiseSpeed_) / (2 * MAX_A * direction_);
        deccelTime_ = cruiseSpeed_ / MAX_A * direction_;

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
//         accelTime_ = abs(error / MAX_A);
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

tuple<double, double, double> TrajectoryCalc::getProfile()
{
    double time = timer_.GetFPGATimestamp().value() - startTime_;
    double tP, tV, tA;

    if (time <= accelTime_ && accelTime_ >= 0.02)
    {
        tA = MAX_A * direction_;
        tV = time * MAX_A * direction_ + initVel_;
        tP = ((tV + initVel_) / 2) * time + initPos_;
    }
    else if (time <= accelTime_ && accelTime_ < 0.02)
    {
        tA = 0;
        tV = initVel_;
        tP = initPos_;
    }
    else if (time > accelTime_ && time <= cruiseTime_ + accelTime_)
    {
        tA = 0;
        tV = cruiseSpeed_;
        tP = accelDist_ + cruiseSpeed_ * (time - accelTime_) + initPos_;
    }
    else if (time > cruiseTime_ + accelTime_ && time < deccelTime_ + accelTime_ + cruiseTime_)
    {
        tA = MAX_A * -direction_;
        tV = cruiseSpeed_ - (time - accelTime_ - cruiseTime_) * MAX_A * direction_;
        tP = accelDist_ + cruiseDist_ + (time - accelTime_ - cruiseTime_) * (cruiseSpeed_ + tV) / 2 + initPos_;
    }
    else if (time > accelTime_ + deccelTime_ + cruiseTime_)
    {
        tA = 0;
        tV = 0;
        tP = setPos_;
    }
    else
    {
        tA = 0;
        tV = 0;
        tP = 0;
    }

    return tuple<double, double, double>(tA, tV, tP);

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
//         tA = MAX_A * direction_;
//         tV = initVel_ + (MAX_A * direction_ * time);
//     }

//     return pair<double, double> (tA, tV);

// }

double TrajectoryCalc::calcPower(double pos, double vel)
{
    tuple<double, double, double> profile = getProfile();

    double error = get<2>(profile) - pos;

    //double absoluteError = (setPos_ - pos);
    //double deltaAbsoluteError = absoluteError - prevAbsoluteError_;
    //prevAbsoluteError_ = absoluteError;
    double velError = get<1>(profile) - vel;
    if(printError_)
    {
        frc::SmartDashboard::PutNumber("TMError", error);
        frc::SmartDashboard::PutNumber("TMVError", velError);
    }
    double kVVolts;
    if(get<1>(profile) == 0)
    {
        kVVolts = 0;
    }
    else
    {
        kVVolts = (abs(get<1>(profile)) - kVI_) * kV_;
        if(get<1>(profile) < 0)
        {
            kVVolts *= -1;
        }
    }

    double power = (kP_ * error) + (kD_ * velError) + kVVolts + (get<0>(profile) * kA_);

    if(abs(power) > GeneralConstants::MAX_VOLTAGE)
    {
        cout << "trapezoidal motion gains are shitting themselves" << endl;
    }

    power = clamp(power, -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE);
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

//     double kVVolts = (abs(profile.second) - kVI_) * kV_;
//     if(profile.second == 0)
//     {
//         kVVolts = 0;
//     }
//     else if(profile.second < 0)
//     {
//         kVVolts *= -1;
//     }

//     double power = (kD_ * error) + (profile.first * kA_) + kVVolts;

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