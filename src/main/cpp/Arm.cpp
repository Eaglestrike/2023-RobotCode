#include "Arm.h"

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
        frc::SmartDashboard::PutNumber("Base Gravity Constant", m_kGravityBot);
        frc::SmartDashboard::PutNumber("Top P", m_pidTop.GetP());
        frc::SmartDashboard::PutNumber("Top I", m_pidTop.GetI());
        frc::SmartDashboard::PutNumber("Top D", m_pidTop.GetD());
        frc::SmartDashboard::PutNumber("Top Gravity Constant", m_kGravityTop);
    }
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::Periodic(){
    //Very basic joint space implementation
    if(m_targetZ < 0.0){//If target is under the floor
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    double targetdz = m_targetZ - m_pivotHeight;
    double distance = sqrt((m_targetX*m_targetX) + (targetdz*targetdz));
    if(distance > m_baseArmLength + m_topArmLength){
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    if(distance < abs(m_baseArmLength - m_topArmLength)){
        frc::SmartDashboard::PutBoolean("Target", false);
        return;
    }
    frc::SmartDashboard::PutBoolean("Target", true);
    double angle = atan2(m_targetX, targetdz);//Angle to target (0 is upwards)
    //Finding ideal angles
    //https://www.google.com/search?q=law+of+cosine
    double a = m_baseArmLength;
    double b = m_topArmLength;
    double c = distance;
    double topArmAng = acos(((a*a)+(b*b)-(c*c))/(2*a*b)); //Angle between 2 arms
    //https://www.google.com/search?q=law+of+sines
    double baseArmAng = M_PI - asin((sin(baseArmAng)/c) * a) - topArmAng; //Angle of base arm relative to target
    double ang1;
    double ang2;
    if(m_targetX > 0){
        ang1 = angle - baseArmAng;
        ang2 = M_PI - topArmAng;
    }
    else{
        ang1 = angle + baseArmAng;
        ang2 = topArmAng - M_PI;
    }
    ang1 = getPrincipalAng2(ang1);
    ang2 = getPrincipalAng2(ang2);

    //Difference of angles (dAng) ~ error
    double baseReading = getAng(m_baseMotor) + m_angOffsetBase;
    double topReading = getAng(m_topMotor) + m_angOffsetTop;

    double dAngBase = getAngDiff(baseReading, ang1);
    double dAngTop = getAngDiff(topReading, ang2);

    double pidBaseOutput = m_pidBase.Calculate(dAngBase) + m_kGravityBot*sin(baseReading);
    double baseVoltage = std::clamp(pidBaseOutput, -m_maxVolts, m_maxVolts);
    //m_baseMotor.SetVoltage(baseVoltage);

    double pidTopOutput = m_pidTop.Calculate(dAngTop) + m_kGravityTop*sin(topReading);
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
        m_kGravityBot = frc::SmartDashboard::GetNumber("Base Gravity Constant", m_kGravityBot);
        m_pidTop.SetP(frc::SmartDashboard::GetNumber("Top P", m_pidTop.GetP()));
        m_pidTop.SetI(frc::SmartDashboard::GetNumber("Top I", m_pidTop.GetI()));
        m_pidTop.SetD(frc::SmartDashboard::GetNumber("Top D", m_pidTop.GetD()));
        m_kGravityTop = frc::SmartDashboard::GetNumber("Top Gravity Constant", m_kGravityTop);
    }
    if(debug){
        frc::SmartDashboard::GetNumber("Target X", m_targetX);
        frc::SmartDashboard::GetNumber("Target Z", m_targetZ);
        frc::SmartDashboard::PutNumber("Base Arm Angle", baseReading);
        frc::SmartDashboard::PutNumber("Top Arm Angle", topReading);
        frc::SmartDashboard::PutNumber("Target Ang Base", ang1);
        frc::SmartDashboard::PutNumber("Target Ang Top", ang2);
        frc::SmartDashboard::PutNumber("Ang Diff Base", dAngBase);
        frc::SmartDashboard::PutNumber("Ang Diff Top", dAngTop);
        frc::SmartDashboard::PutNumber("Base Voltage", baseVoltage);
        frc::SmartDashboard::PutNumber("Top Voltage", topVoltage);
    }
}

void Arm::DisabledPeriodic(){
    double baseReading = getAng(m_baseMotor) + m_angOffsetBase;
    double topReading = getAng(m_topMotor) + m_angOffsetTop;
     if(configDimensions){
        m_angOffsetBase = frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
    }
    if(debug){
        frc::SmartDashboard::GetNumber("Target X", m_targetX);
        frc::SmartDashboard::GetNumber("Target Z", m_targetZ);
        frc::SmartDashboard::PutNumber("Base Arm Angle", baseReading);
        frc::SmartDashboard::PutNumber("Top Arm Angle", topReading);
    }
}

void Arm::setTarget(double targetX, double targetZ){
    m_targetX = targetX;
    m_targetZ = targetZ;
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::moveTarget(double dx, double dz){
    m_targetX += dx;
    m_targetZ += dz;
    m_pidBase.Reset();
    m_pidTop.Reset();
}