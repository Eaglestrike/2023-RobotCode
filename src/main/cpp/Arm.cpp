#include "Arm.h"

void Arm::init(){
    if(configDimensions){
        frc::SmartDashboard::PutNumber("Base Length", m_baseArmLength);
        frc::SmartDashboard::PutNumber("Top Length", m_topArmLength);
        frc::SmartDashboard::PutNumber("Pivot Height", m_pivotHeight);
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
}

void Arm::periodic(){
    //Very basic joint space implementation
    if(m_targetZ < 0.0){//If target is under the floor
        return;
    }
    double targetdy = m_pivotHeight - m_targetZ;
    double distance = sqrt((m_targetX*m_targetX) + (targetdy*targetdy));
    if(distance > m_baseArmLength + m_topArmLength){
        return;
    }
    //Finding ideal angles
    //https://www.google.com/search?q=law+of+cosine
    double a = m_baseArmLength;
    double b = m_topArmLength;
    double c = distance;
    double baseArmAng = acos(((c*c)-(a*a)-(b*b))/(2*a*b)); //Angle between 2 arms
    //https://www.google.com/search?q=law+of+sines
    double topArmAng = (sin(baseArmAng)/c) * b; //Angle of base arm

    //Difference of angles (dAng) ~ error
    double dAngBase = getAngDiff(getAng(m_baseMotor), baseArmAng);
    double dAngTop = getAngDiff(getAng(m_baseMotor), topArmAng);

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
    }
    if(configPID){
        m_maxVolts = frc::SmartDashboard::GetNumber("Max Volts", m_maxVolts);
        m_pidBase.SetP(frc::SmartDashboard::GetNumber("Base P", m_pidBase.GetP()));
        m_pidBase.SetI(frc::SmartDashboard::GetNumber("Base I", m_pidBase.GetI()));
        m_pidBase.SetD(frc::SmartDashboard::GetNumber("Base D", m_pidBase.GetD()));
        m_pidTop.SetP(frc::SmartDashboard::GetNumber("Top P", m_pidTop.GetP()));
        m_pidTop.SetI(frc::SmartDashboard::GetNumber("Top I", m_pidTop.GetI()));
        m_pidTop.SetD(frc::SmartDashboard::GetNumber("Top D", m_pidTop.GetD()));
    }
}

void Arm::setTarget(double targetX, double targetZ){
    m_targetX = targetX;
    m_targetZ = targetZ;
    m_pidBase.Reset();
    m_pidTop.Reset();
}