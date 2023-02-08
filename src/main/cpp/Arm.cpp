#include "Arm.h"
using namespace Helpers;

// Sets up SmartDashboard, zeroes motor readings.
void Arm::init(){
    if (configDimensions) {
        frc::SmartDashboard::PutNumber("Base Length", m_baseArmLength);
        frc::SmartDashboard::PutNumber("Top Length", m_topArmLength);
        frc::SmartDashboard::PutNumber("Pivot Height", m_pivotHeight);
    }
    if (configPID) {
        frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
        frc::SmartDashboard::PutNumber("Max Volts", m_maxVolts);
        frc::SmartDashboard::PutNumber("Base P", m_pidBase.GetP());
        frc::SmartDashboard::PutNumber("Base I", m_pidBase.GetI());
        frc::SmartDashboard::PutNumber("Base D", m_pidBase.GetD());
        frc::SmartDashboard::PutNumber("Base Gravity Constant", m_kGravityBot);
        frc::SmartDashboard::PutNumber("Top P", m_pidTop.GetP());
        frc::SmartDashboard::PutNumber("Top I", m_pidTop.GetI());
        frc::SmartDashboard::PutNumber("Top D", m_pidTop.GetD());
        frc::SmartDashboard::PutNumber("Top Gravity Constant", m_kGravityTop);
        frc::SmartDashboard::PutNumber("Bot Slack", m_botArmSlack);
        frc::SmartDashboard::PutNumber("Top Slack", m_topArmSlack);
    }
    //frc::SmartDashboard::PutNumber("Target Ang Base", 0);
    //frc::SmartDashboard::PutNumber("Target Ang Top", 0);

    m_baseMotor.SetNeutralMode(NeutralMode::Brake);
    m_baseMotor2.SetNeutralMode(NeutralMode::Brake);
    //m_baseMotor2.SetInverted(InvertType::FollowMaster);
    //m_baseMotor2.Follow(m_baseMotor);

    m_topMotor.SetNeutralMode(NeutralMode::Brake);
    m_topMotor2.SetNeutralMode(NeutralMode::Brake);
    m_topMotor2.Follow(m_topMotor);

    resetEncoder();

    setBrakes(false, false);

    m_pidBase.SetIntegratorRange(-m_maxVolts, m_maxVolts);
    m_pidTop.SetIntegratorRange(-m_maxVolts, m_maxVolts);
}

void Arm::resetEncoder(){
    m_baseMotor.SetSelectedSensorPosition(0);
    m_topMotor.SetSelectedSensorPosition(0);
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::Periodic(){
    ReadSmartDashboard();

    setBrakes(false, false);
    baseReading = getAng(m_baseMotor, ArmConstants::BASE_GEAR_RATIO) + m_angOffsetBase;
    topReading = getAng(m_topMotor, ArmConstants::TOP_GEAR_RATIO) + m_angOffsetTop;

    if(debug){
        frc::SmartDashboard::PutNumber("Base Arm Angle", baseReading);
        frc::SmartDashboard::PutNumber("Top Arm Angle", topReading);
    }
    if(debug){
        frc::SmartDashboard::PutNumber("Target X", m_targetX);
        frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
    }
}

void Arm::setBrakes(bool base, bool top){
    m_baseBrake.Set(!base);
    m_topBrake.Set(!top);
}

void Arm::TargetFail(){
    frc::SmartDashboard::PutBoolean("Target", false);
    m_topMotor.SetVoltage(units::volt_t{m_kGravityTop*sin(topReading)});
    m_baseMotor.SetVoltage(units::volt_t{-m_kGravityBot*sin(baseReading)});
}

// Main execution loop of the arm, runs during teleop.
void Arm::TeleopPeriodic() {
    // Very basic joint space implementation

    //Counter Slack
    // if(baseReading < 0){
    //     baseReading += m_botArmSlack;
    // }
    // if(topReading > 0){
    //     topReading += m_topArmSlack;
    // }

    // If target is under floor
    if (m_targetZ < 0.0) {
        std::cout << "Under the floor" << std::endl;
        TargetFail();
        return;
    }
    double targetdz = m_targetZ - m_pivotHeight;
    double distance = sqrt((m_targetX*m_targetX) + (targetdz*targetdz));
    
    // Target distance is too far, signal that to driver and cancel movement
    if(distance > m_baseArmLength + m_topArmLength){
        std::cout << "Too Far" << std::endl;
        TargetFail();
        return;
    }

    // Target distance too close, signal to driver and cancel movement
    if(distance < abs(m_baseArmLength - m_topArmLength)){
        TargetFail();
        std::cout<<"Too Close"<<std::endl;
        return;
    }
    double angle = atan2(m_targetX, targetdz);//Angle to target (0 is upwards)
    //Finding ideal angles
    //https://www.google.com/search?q=law+of+cosine
    double a = m_baseArmLength;
    double b = m_topArmLength;
    double c = distance;
    double topArmAng = acos(((a*a)+(b*b)-(c*c))/(2*a*b)); //Angle between 2 arms
    //https://www.google.com/search?q=law+of+sines
    double baseArmAng = M_PI - asin((sin(topArmAng)/c) * a) - topArmAng; //Angle of base arm relative to target
    double ang1;
    double ang2;

    // Handling cases when the target is front of or behind the arm
    // Including the case where the elbow bend is facing down, like ^
    if (m_targetX > 0) {
        ang1 = angle - baseArmAng;
        ang2 = M_PI - topArmAng + ang1;
    } else {
        ang1 = angle + baseArmAng;
        ang2 = topArmAng - M_PI + ang1;
    }
    ang1 = getPrincipalAng2(ang1);
    ang2 = getPrincipalAng2(ang2);

    //Difference of angles (dAng) ~ error

    //ang1 = frc::SmartDashboard::GetNumber("Target Ang Base", baseReading);
    //ang2 = frc::SmartDashboard::GetNumber("Target Ang Top", topReading);

    if(angInBetween(ang1, ArmConstants::BASE_MIN_ANG, ArmConstants::BASE_MAX_ANG)){
        TargetFail();
        std::cout<<"Base Can't Reach"<<std::endl;
        return;
    }

    if(angInBetween(ang2, ArmConstants::TOP_MIN_ANG, ArmConstants::TOP_MAX_ANG)){
        TargetFail();
        std::cout<<"Top Can't Reach"<<std::endl;
        return;
    }

    double dAngBase = getAngDiff(ang1, baseReading);
    double dAngTop = getAngDiff(ang2, topReading);

    if(angInBetween(ArmConstants::BASE_MIN_ANG, baseReading, ang1)){
        dAngBase = 2*M_PI - dAngBase;
    }

    if(angInBetween(ArmConstants::TOP_MIN_ANG, topReading, ang2)){
        dAngTop = 2*M_PI - dAngTop;
    }

    dAngBase = scaleError(2.1, 0.07, dAngBase);
    dAngTop = scaleError(2.1, 0.07, dAngTop);

    double pidBaseOutput = m_pidBase.Calculate(dAngBase) - m_kGravityBot*sin(baseReading);
    double baseVoltage = std::clamp(pidBaseOutput, -m_maxVolts, m_maxVolts);
    m_baseMotor.SetVoltage(units::volt_t{baseVoltage});

    double pidTopOutput = m_pidTop.Calculate(dAngTop) - m_kGravityTop*sin(topReading);
    double topVoltage = std::clamp(pidTopOutput, -m_maxVolts, m_maxVolts);
    m_topMotor.SetVoltage(units::volt_t{topVoltage});

    frc::SmartDashboard::PutBoolean("Target", true);
    frc::SmartDashboard::PutNumber("Target Distance", distance);
    if(configPID){
        m_angOffsetBase = frc::SmartDashboard::GetNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::GetNumber("Top Ang Offset", m_angOffsetTop);
    }
    if (debug) {
        frc::SmartDashboard::PutNumber("Target X", m_targetX);
        frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
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

// Turns the motors off
void Arm::DisabledInit() {
    m_baseMotor.SetNeutralMode(NeutralMode::Brake);
    m_topMotor.SetNeutralMode(NeutralMode::Brake);
    m_baseMotor.SetVoltage(units::volt_t{0});
    m_topMotor.SetVoltage(units::volt_t{0});
}

void Arm::DisabledPeriodic(){
    if(configPID){
        m_angOffsetBase = frc::SmartDashboard::GetNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::GetNumber("Top Ang Offset", m_angOffsetTop);
    }
    //frc::SmartDashboard::PutNumber("Ticks Bottom", m_baseMotor.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("Ticks Top", m_topMotor.GetSelectedSensorPosition());
}

void Arm::TestPeriodic(double vBase, double vTop){
    if(configPID){
        frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
    }

    m_baseMotor.SetVoltage(units::volt_t{vBase});
    m_baseMotor2.SetVoltage(units::volt_t{vBase});
    m_topMotor.SetVoltage(units::volt_t{vTop});

    if(debug){
        frc::SmartDashboard::PutNumber("Base Voltage", vBase);
        frc::SmartDashboard::PutNumber("Top Voltage", vTop);
        frc::SmartDashboard::PutNumber("Target X", m_targetX);
        frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
    }
}

void Arm::ReadSmartDashboard(){
    if (configDimensions) {
        m_baseArmLength = frc::SmartDashboard::GetNumber("Base Length", m_baseArmLength);
        m_topArmLength = frc::SmartDashboard::GetNumber("Top Length", m_topArmLength);
        m_pivotHeight = frc::SmartDashboard::GetNumber("Pivot Height", m_pivotHeight);
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
        m_botArmSlack = frc::SmartDashboard::GetNumber("Bot Gravity Slack", m_botArmSlack);
    }
}

/**
 * @brief Sets the target to new values, resets PID to accomodate new values.
 * 
 * @param targetX the new x value
 * @param targetZ the new z value
 */
void Arm::setTarget(double targetX, double targetZ) {
    // TODO: Add error checking here, or in a new function.
    m_targetX = targetX;
    m_targetZ = targetZ;
    m_pidBase.Reset();
    m_pidTop.Reset();
}

/**
 * @brief Moves the target based on the passed in parameters, resets PID to accomodate new values.
 * 
 * @param dx how much the x value will be changed
 * @param dz how much the z value will be changed
 */
void Arm::moveTarget(double dx, double dz) {
    m_targetX += dx;
    m_targetZ += dz;
    m_pidBase.Reset();
    m_pidTop.Reset();
}

/**
 * @brief Ideally will be called by driverstation to reset the target, for arm positioning purposes. Also resets PID to recalculate for new values.
 */
void Arm::resetTarget() {
    m_targetX = 0.0;
    m_targetZ = m_baseArmLength + m_topArmLength + m_pivotHeight;
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::changeOffsetBase(double da) {
    m_angOffsetBase += da;
    m_pidBase.Reset();
}

void Arm::changeOffsetTop(double da) {
    m_angOffsetTop += da;
    m_pidBase.Reset();
}

void Arm::resetOffsets(){
    m_angOffsetTop = 0.0;
    m_angOffsetBase = 0.0;
}