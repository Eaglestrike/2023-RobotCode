#include "Drivebase/SwerveModule.h"

#include "Helpers/GeometryHelper.h"
#include "Drivebase/SwerveConstants.h"
using namespace SwerveConstants::ModuleConstants;
using namespace Poses;

SwerveModule::SwerveModule(int turnID, int driveID, int cancoderID, double offset) :
    turnMotor_(turnID, "drivebase"),
    driveMotor_(driveID, "drivebase"),
    cancoder_(cancoderID, "drivebase"),
    trajectoryCalc_({maxV, maxA, kP, kD, kV, kA, kVI}), offset_(offset)
{
    turnMotor_.SetInverted(TalonFXInvertType::CounterClockwise);
    driveMotor_.SetInverted(TalonFXInvertType::Clockwise);

    turnMotor_.SetNeutralMode(NeutralMode::Brake);
    driveMotor_.SetNeutralMode(NeutralMode::Brake);

    id_ = std::to_string(driveID);

    initTrajectory_ = false;
    cancoder_.ClearStickyFaults(); 
}

/// @brief Collects module data
void SwerveModule::periodic(){
    double angle = cancoder_.GetAbsolutePosition() + offset_; // Degrees
    angle = GeometryHelper::getPrincipalAng2Deg(angle);
    double speed = (driveMotor_.GetSelectedSensorVelocity() / 2048.0) * 10.0 * SwerveConstants::DRIVE_GEAR_RATIO * 2.0 * M_PI * SwerveConstants::TREAD_RADIUS;

    currPose_.ang = angle;
    currPose_.mag = speed;
}

/// @brief Moves
/// @param target target pose
/// @param inVolts direct volts or target velocity
void SwerveModule::move(ModulePose target, bool inVolts)
{
    double time = timer_.GetFPGATimestamp().value();
    // frc::SmartDashboard::PutNumber(id_ + " time", time);
    // frc::SmartDashboard::PutNumber(id_ + " sp", driveSpeed);
    // frc::SmartDashboard::PutNumber(id_ + " ang", angle);
    dT_ = time - prevTime_;
    prevTime_ = time;

    //frc::SmartDashboard::PutNumber(id_ + " Wanted speed", driveSpeed);
    //frc::SmartDashboard::PutNumber(id_ + " Wanted angle", angle);

    /*if(abs(setTrajectoryPos_ - angle) > 0.1 && initTrajectory_) //COULDO get value
    {
        setTrajectoryPos_ = angle;

        //double pos = get<2>(trajectoryCalc_.getProfile()) + posOffset_;
        //posOffset_ = pos;
        double pos = getAngle();
        posOffset_ = pos;

        double error = findError(angle, pos);

        //double vel = get<1>(trajectoryCalc_.getProfile());
        double vel = cancoder_.GetVelocity();

        trajectoryCalc_.generateTrajectory(0, error, vel);
    }
    else if(!initTrajectory_)
    {
        initTrajectory_ = true;
        setTrajectoryPos_ = angle;
        posOffset_ = getAngle();

        double error = findError(angle, getAngle());

        trajectoryCalc_.generateTrajectory(0, error, cancoder_.GetVelocity());
    }
    
    if(initTrajectory_)
    {
        double pos = getAngle() - posOffset_;
        Helpers::normalizeAngle(pos);
        double vel = cancoder_.GetVelocity();

        units::volt_t swivelVols(trajectoryCalc_.calcPower(pos, vel));
        turnMotor_.SetVoltage(swivelVols);
    }*/

    //turnMotor_.SetVoltage(units::volt_t(4));

    // double rawError = angle - getAngle();
    // if(abs(rawError) > 90)
    // {
    //     direction_ = -1;
    // }
    // else
    // {
    //     direction_ = 1;
    // }

    //OLD?
    //1, 3.77, 95.49, 95.49
    //2, 0.91, 395.60, 197.8
    //3, 0.55, 654.54, 218.18
    //4, 0.4, 900, 225
    //frc::SmartDashboard::PutNumber(id_ + " pos", cancoder_.GetAbsolutePosition());
    //frc::SmartDashboard::PutNumber(id_ + " speed", driveMotor_.GetSelectedSensorVelocity());
    //Loadless idk
    //0.8, 300 for right, 350 for left
    //1, 900 right, 700 left
    //1.5, 1806bl,1980
    //2, 2950
    //2.5, 3850
    //3, 4850
    //3.5, 5775
    //4, 6700
    //4.5, 7660
    //5, 8600
    //5.5, 9600
    //6, 10475
    //6.5, 11450
    //7, 12400

    //Robot
    //0.8, .05
    //1, 0.15
    //1.5, .39
    //2, .62
    //2.5, .9
    //3, 1.16
    //3.5, 1.4
    //4, 
    //4.5, 
    //5, 
    //5.5, 
    //6, 
    //6.5, 

    units::volt_t turnVolts{calcAngPID(target.ang)};
    turnMotor_.SetVoltage(turnVolts);

    // if(abs(driveSpeed) < 0.1)
    // {
    //     driveMotor_.SetVoltage(units::volt_t(0));
    // }
    // else
    // {
    //     driveMotor_.SetVoltage(units::volt_t(direction_ * frc::SmartDashboard::GetNumber("Swerve Volts", 0)));
    // }
    // frc::SmartDashboard::PutNumber(id_ + " vel", (driveMotor_.GetSelectedSensorVelocity() / 2048.0) * 10 * SwerveConstants::DRIVE_GEAR_RATIO * 2 * M_PI * SwerveConstants::TREAD_RADIUS);
    
    if(inVolts)
    {
        driveMotor_.SetVoltage(units::volt_t(direction_ * target.mag));
    }
    else
    {
        units::volt_t driveVolts{direction_ * calcDrivePID(target.mag)};
        driveMotor_.SetVoltage(driveVolts);
    }

    //Turn with arm
    //0.74, 0
    //0.8, 0.03
    //1, 0.125
    //2, 0.59
    //3, 1.1
    //4, 1.54
    //5, 1.98
}

//1, 107
//2, 400
//3, 666
//4, 921
//5, 1180
//6, 1440p
//7, 1700
//8, 1960
//9, 2230
//10, 2500
//11, 2730
//12, 2855

/// @brief PID controller
/// @param setAngle target angle
/// @return volts
double SwerveModule::calcAngPID(double setAngle)
{

    double error = findError(setAngle, currPose_.ang);
    //frc::SmartDashboard::PutNumber(id_ + "Error", error);

    aIntegralError_ += error * dT_;
    double deltaError = (error - aPrevError_) / dT_;
    if(abs(aPrevError_) < 2 && error > 5) //COULDO get value
    {
        deltaError = 0;
        aIntegralError_ = 0;
    } 
    aPrevError_ = error;

    double power = (akP_*error) + (akI_*aIntegralError_) + (akD_*deltaError); //COULDO implement integral anti-windup or just don't use kI

    //return 1.5;
    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);
}

double SwerveModule::calcDrivePID(double driveSpeed)
{
    // double velocity = (driveSpeed * GeneralConstants::MAX_RPM * GeneralConstants::TICKS_PER_ROTATION) / 600;
    // double error = velocity - driveMotor_.GetSelectedSensorVelocity(); //This is probably wrong

    // dIntegralError_ += error * dT_;
    // double deltaError = (error - dPrevError_) / dT_;
    // dPrevError_ = error;

    // if(abs(deltaError) < 40 && error > 100) //COULDO get value, also test if needed
    // {
    //     deltaError = 0;
    //     dIntegralError_ = 0;
    // }

    double feedForward = GeneralConstants::MAX_VOLTAGE * driveSpeed;
    // double feedForward;
    // if(driveSpeed == 0)
    // {
    //     feedForward = 0;
    // }
    // else
    // {
    //     feedForward = ((driveSpeed * SwerveConstants::MAX_TELE_VEL) - SwerveConstants::klVI) / SwerveConstants::klV;
    // }
    

    //double power = (dkP_*error) + (dkI_*aIntegralError_) + (dkD_*deltaError) + feedForward;
    double power = feedForward;

    return std::clamp(power, -(double)GeneralConstants::MAX_VOLTAGE, (double)GeneralConstants::MAX_VOLTAGE);

}

double SwerveModule::findError(double setAngle, double angle)
{
    double rawError = GeometryHelper::getAngDiffDeg(angle, setAngle);

    if(abs(rawError) > 90.0)
    {
        direction_ = -1.0;
        if(rawError > 0){
            return rawError - 180.0;
        }
        else{
            return rawError + 180.0;
        }
    }
    else
    {
        direction_ = 1.0;
        return rawError;
    }
}

/**
 * Returns velocity of the wheels
*/
Vector SwerveModule::getVelocity(){
    return Vector::extendDeg(currPose_.mag, currPose_.ang);
};