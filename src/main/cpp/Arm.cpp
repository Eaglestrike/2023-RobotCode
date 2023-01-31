#include "Arm.h"
#include "FeedForward.h"


#include <iostream>

void Arm::init(){
    if(configDimensions){
        frc::SmartDashboard::PutNumber("Base Length", m_baseArmLength);
        frc::SmartDashboard::PutNumber("Top Length", m_topArmLength);
        frc::SmartDashboard::PutNumber("Pivot Height", m_pivotHeight);
        frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
    }
    if(configPID){
        frc::SmartDashboard::PutNumber("Max Volts", m_maxVolts);
        frc::SmartDashboard::PutNumber("Base P", m_pidBase.GetP());
        frc::SmartDashboard::PutNumber("Base I", m_pidBase.GetI());
        frc::SmartDashboard::PutNumber("Base D", m_pidBase.GetD());
        frc::SmartDashboard::PutNumber("Top P", m_pidTop.GetP());
        frc::SmartDashboard::PutNumber("Top I", m_pidTop.GetI());
        frc::SmartDashboard::PutNumber("Top D", m_pidTop.GetD());
    }
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::periodic(){
    //Very basic joint space implementation
    if(m_targetZ < 0.0){//If target is under the floor
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    double targetdz = m_pivotHeight - m_targetZ;
    double distance = sqrt((m_targetX*m_targetX) + (targetdz*targetdz));
    if(distance > m_baseArmLength + m_topArmLength){
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    if(distance < abs(m_baseArmLength - m_topArmLength) || distance == 0){
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    frc::SmartDashboard::PutBoolean("Target", true);
    double angle = atan2(targetdz, m_targetX);//Angle to target
    //Finding ideal angles
    //https://www.google.com/search?q=law+of+cosine
    double a = m_baseArmLength;
    double b = m_topArmLength;
    double c = distance;
    double topArmAng = acos(((c*c)-(a*a)-(b*b))/(2*a*b)); //Angle between 2 arms
    //https://www.google.com/search?q=law+of+sines
    double baseArmAng = asin((sin(baseArmAng)/c) * b); //Angle of base arm
    if(m_targetX > 0){
        topArmAng = -topArmAng;
    }
    else{
        baseArmAng = -baseArmAng;
    }

    //frc::SmartDashboard::PutNumber("Base Ang", baseArmAng);

    auto traj = trajManager.get_traj(units::angle::radian_t(getAng(m_baseMotor) + m_angOffsetBase), 
        units::angle::radian_t(baseArmAng + angle),
        units::angle::radian_t(getAng(m_topMotor) + m_angOffsetTop),
        units::angle::radian_t(baseArmAng + angle));
    auto ffu_volts = FeedForward::getffu(traj);

    std::cout << "volts: " << ffu_volts[0] << ", " << ffu_volts[1] << "\n";

    frc::SmartDashboard::PutNumber("base voltage", ffu_volts[0]);
    frc::SmartDashboard::PutNumber("top voltage", ffu_volts[1]);

    double dAngBase = getAngDiff(getAng(m_baseMotor), baseArmAng + angle);
    double dAngTop = getAngDiff(getAng(m_topMotor), topArmAng);

   // std::cout << "get Ang bottom " <<  getAng(m_baseMotor) << ", base arm: " << baseArmAng << ", angle: " <<  angle << "\n";

    frc::SmartDashboard::PutNumber("dAngBase", dAngBase);
    frc::SmartDashboard::PutNumber("dAngTop", dAngTop);

    frc::SmartDashboard::PutNumber("base angle", m_baseMotor.GetSelectedSensorPosition() / 4096 * 2*M_PI);
    frc::SmartDashboard::PutNumber("top angle", (m_topMotor.GetSelectedSensorPosition() - m_baseMotor.GetSelectedSensorPosition()) / 4096 * 2*M_PI);

    double baseVoltage = std::clamp(ffu_volts[0], -m_maxVolts, m_maxVolts);
    m_baseMotor.SetVoltage(units::voltage::volt_t(baseVoltage));

    double topVoltage = std::clamp(ffu_volts[1], -m_maxVolts, m_maxVolts);
    m_topMotor.SetVoltage(units::voltage::volt_t(topVoltage));

    return;

    /*//Difference of angles (dAng) ~ error
    double dAngBase = getAngDiff(getAng(m_baseMotor) + m_angOffsetBase, baseArmAng + angle);
    double dAngTop = getAngDiff(getAng(m_baseMotor) + m_angOffsetTop, topArmAng);

    double pidBaseOutput = m_pidBase.Calculate(dAngBase);
    double baseVoltage = std::clamp(pidBaseOutput, -m_maxVolts, m_maxVolts);
    //m_baseMotor.SetVoltage(baseVoltage);

    double pidTopOutput = m_pidTop.Calculate(dAngTop);
    double topVoltage = std::clamp(pidTopOutput, -m_maxVolts, m_maxVolts);
    //m_topMotor.SetVoltage(topVoltage);

    if(configDimensions){
        m_baseArmLength = frc::SmartDashboard::GetNumber("Base Length", m_baseArmLength);
        m_topArmLength = frc::SmartDashboard::GetNumber("Top Length", m_topArmLength);
        m_pivotHeight = frc::SmartDashboard::GetNumber("Pivot Height", m_pivotHeight);
        m_angOffsetBase = frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
    }
    if(configPID){
        m_maxVolts = frc::SmartDashboard::GetNumber("Max Volts", m_maxVolts);
        m_pidBase.SetP(frc::SmartDashboard::GetNumber("Base P", m_pidBase.GetP()));
        m_pidBase.SetI(frc::SmartDashboard::GetNumber("Base I", m_pidBase.GetI()));
        m_pidBase.SetD(frc::SmartDashboard::GetNumber("Base D", m_pidBase.GetD()));
        m_pidTop.SetP(frc::SmartDashboard::GetNumber("Top P", m_pidTop.GetP()));
        m_pidTop.SetI(frc::SmartDashboard::GetNumber("Top I", m_pidTop.GetI()));
        m_pidTop.SetD(frc::SmartDashboard::GetNumber("Top D", m_pidTop.GetD()));
    }*/
}

void Arm::setTarget(double targetX, double targetZ){
    m_targetX = targetX;
    m_targetZ = targetZ;
    m_pidBase.Reset();
    m_pidTop.Reset();
}