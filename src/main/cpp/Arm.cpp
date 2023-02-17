#include "Arm.h"
#include <frc/trajectory/TrapezoidProfile.h>
#include <stdlib.h>
#include "Robot.h"


using namespace Helpers;
using namespace ctre::phoenixpro;

// Sets up SmartDashboard, zeroes motor readings.
void Arm::init(Robot* robot){
    this->robot = robot;
    if (configDimensions) {
        frc::SmartDashboard::PutNumber("Base Length", m_baseArmLength);
        frc::SmartDashboard::PutNumber("Top Length", m_topArmLength);
        frc::SmartDashboard::PutNumber("Pivot Height", m_pivotHeight);
    }
    if (configPID) {
        frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
        frc::SmartDashboard::PutNumber("Max Amps Bot", m_maxAmpsBot);
        frc::SmartDashboard::PutNumber("Max Amps Top", m_maxAmpsTop);
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

    //TODO: figure out neutral mode
    //m_baseMotor.SetNeutralMode(NeutralMode::Brake);
    //m_baseMotor2.SetNeutralMode(NeutralMode::Brake);

    //m_baseMotor2.SetInverted(InvertType::FollowMaster);
    //m_baseMotor2.Follow(m_baseMotor);
    

    //TODO: figue out neutral mode
    //m_topMotor.SetNeutralMode(NeutralMode::Brake);
    //m_topMotor2.SetNeutralMode(NeutralMode::Brake);

    //m_topMotor2.Follow(m_topMotor);

    resetEncoder();

    setBrakes(false, false);

    m_pidBase.SetIntegratorRange(-m_maxAmpsBot, m_maxAmpsBot);
    m_pidTop.SetIntegratorRange(-m_maxAmpsTop, m_maxAmpsTop);

    frc::SmartDashboard::PutNumber("Target X", m_targetX);
    frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
}

void Arm::resetEncoder(){
    m_baseMotor.SetRotorPosition(0_deg);
    m_topMotor.SetRotorPosition(0_deg);
    m_pidBase.Reset();
    m_pidTop.Reset();
}

void Arm::Periodic(){
    ReadSmartDashboard();

    setBrakes(false, false);
    baseReading = getAng(m_baseMotor, ArmConstants::BASE_GEAR_RATIO) + m_angOffsetBase;
    topReading = getAng(m_topMotor, ArmConstants::TOP_GEAR_RATIO) + m_angOffsetTop + 30.0/54.0*baseReading;

    if (baseReading < -M_PI || baseReading > M_PI || topReading < -0.2 || topReading > 2*M_PI) {
        // EMERGENCY STOP
        // NO MATTER WHAT JUST STOP
        //abort();
        //exit(1);
        m_baseMotor.SetControl(units::current::ampere_t(0));
        m_baseMotor2.SetControl(units::current::ampere_t(0));
        m_topMotor.SetControl(units::current::ampere_t(0));
        m_topMotor2.SetControl(units::current::ampere_t(0));
        robot->emergencyStop = true;
        std::cout << "EMERGENCY ALERT" << std::endl;
        return;
    }

    if(debug){
        frc::SmartDashboard::PutNumber("Base Arm Angle", baseReading);
        frc::SmartDashboard::PutNumber("Top Arm Angle", topReading);
    }
    m_targetX = frc::SmartDashboard::GetNumber("Target X", m_targetX);
    m_targetZ = frc::SmartDashboard::GetNumber("Target Z", m_targetZ);
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

    //setTarget(0.7112, 1.143);

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
    double topArmAng = ((a*a)+(b*b)-(c*c))/(2*a*b); //Angle between 2 arms
    if(topArmAng > 1.0){
        topArmAng = 0;
    }
    else if(topArmAng < -1.0){
        topArmAng = M_PI;
    }
    else{
        topArmAng = acos(topArmAng);
    }
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
    
    //std::cout<< "baseArmAng: " << baseArmAng << ", ang1: " << ang1 << ", ang2: " << ang2 << std::endl;
    //Difference of angles (dAng) ~ error

    //ang1 = frc::SmartDashboard::GetNumber("Target Ang Base", baseReading);
    //ang2 = frc::SmartDashboard::GetNumber("Target Ang Top", topReading);

    double dAngBase = getAngDiff(ang1, baseReading);
    double dAngTop = getAngDiff(ang2, topReading);

    if(debug){
        frc::SmartDashboard::PutNumber("Base Arm Angle", baseReading);
        frc::SmartDashboard::PutNumber("Top Arm Angle", topReading);
        frc::SmartDashboard::PutNumber("Target Ang Base", ang1);
        frc::SmartDashboard::PutNumber("Target Ang Top", ang2);
        frc::SmartDashboard::PutNumber("Ang Error (Diff) Base", dAngBase);
        frc::SmartDashboard::PutNumber("Ang Error (Diff) Top", dAngTop);
    }

    if(!angInBetween(ang1, ArmConstants::BASE_MIN_ANG, ArmConstants::BASE_MAX_ANG)){
        TargetFail();
        std::cout<<"Base Can't Reach"<<std::endl;
        return;
    }

    if(angInBetween(ang2, ArmConstants::TOP_MIN_ANG, ArmConstants::TOP_MAX_ANG)){
        TargetFail();
        std::cout<<"Top Can't Reach"<<std::endl;
        return;
    }

    if(angInBetween(ArmConstants::BASE_MIN_ANG, baseReading, ang1)){
        if(dAngBase > 0){
            dAngBase = -2*M_PI + dAngBase;
        }
        else{
            dAngBase = 2*M_PI + dAngBase;
        }
    }

   // std::cout<< "Top Min Ang:" <<ArmConstants::TOP_MIN_ANG << ", top reading:" << topReading << ", ang reading:" <<ang2 << std::endl;
    if(angInBetween(ArmConstants::TOP_MIN_ANG, topReading, ang2)){
        if(dAngTop > 0){
            dAngTop = -2*M_PI + dAngTop;
        }
        else{
            dAngTop = 2*M_PI + dAngTop;
        }
    }

    dAngBase = scaleError(2.1, 0.07, dAngBase);
    dAngTop = scaleError(2.1, 0.07, dAngTop);

    double baseTarget = dAngBase + baseReading;
    double topTarget = dAngTop + topReading;
    if (firstRun) {
        baseArmProfile = frc::TrapezoidProfile<units::radians>(
            frc::TrapezoidProfile<units::angle::radians>::Constraints{0.1_rad_per_s, 0.03_rad_per_s_sq},
            frc::TrapezoidProfile<units::angle::radians>::State{units::angle::radian_t(baseTarget), 0_rad_per_s},
            frc::TrapezoidProfile<units::angle::radians>::State{units::angle::radian_t(baseReading), 0_rad_per_s}
        );
        topArmProfile = frc::TrapezoidProfile<units::radians>(
            frc::TrapezoidProfile<units::angle::radians>::Constraints{0.1_rad_per_s, 0.03_rad_per_s_sq},
            frc::TrapezoidProfile<units::angle::radians>::State{units::angle::radian_t(topTarget), 0_rad_per_s},
            frc::TrapezoidProfile<units::angle::radians>::State{units::angle::radian_t(topReading), 0_rad_per_s}
        );
        startTime = frc::Timer::GetFPGATimestamp();
        firstRun = false;
    }

    auto state = baseArmProfile.Calculate(frc::Timer::GetFPGATimestamp() - startTime);
    double baseArmPos = state.position.to<double>();
    double baseArmVel = state.velocity.to<double>();
    std::cout << "vel: " << baseArmVel << "pos: " << baseArmPos << std::endl; 
    std::cout << "base reading: " << baseReading << std::endl;
    double baseArmAccel = (baseArmVel - baseArmLastVel) / 0.02;
    baseArmLastVel = baseArmVel;

    state = topArmProfile.Calculate(frc::Timer::GetFPGATimestamp() - startTime);
    double topArmPos = state.position.to<double>();
    double topArmVel = state.velocity.to<double>();
    double topArmAccel = (topArmVel - topArmLastVel) / 0.02;
    topArmLastVel = topArmVel;


    double baseArmGravff = std::sin(baseReading) * m_base_r * m_base_m * 9.81;
    double baseArmAccelff = baseArmAccel * m_base_I;
    // feed forward base arm
    double baseArmTorque = baseArmGravff + baseArmAccelff;
    frc::SmartDashboard::PutNumber("base arm torque", baseArmTorque);
    double feedForwardBase = baseArmTorque / m_base_Kt;

    // feed forward top arm
    double topArmTorque = std::sin(topReading) * m_top_r * m_top_m * 9.81 + topArmAccel * m_top_I;
    frc::SmartDashboard::PutNumber("top arm torque", topArmTorque);
    double feedForwardTop = topArmTorque / m_top_Kt;

    // also need to factor in centripetal acceleration:
    //sin(baseReading-topReading) * topArmVel^2 * top_arm_mass / top_arm_length

    m_baseMotor2.SetControl(controls::Follower(m_baseMotor.GetDeviceID(), false));
    m_topMotor2.SetControl(controls::Follower(m_topMotor.GetDeviceID(), false));

    double pidBaseOutput = m_pidBase.Calculate(dAngBase);
    double add = 10;
    double baseCurrent = feedForwardBase;//std::clamp(pidBaseOutput+feedForwardBase, -m_maxAmpsBot, m_maxAmpsBot);
    if (baseCurrent < 0) {
        baseCurrent -= add;
    } else if (baseCurrent > 0) {
        baseCurrent += add;
    }

    //m_baseMotor.SetVoltage(units::volt_t{baseVoltage});
    //m_baseMotor2.SetVoltage(units::volt_t{baseVoltage});
    if (baseCurrent < 0) {
        m_baseMotor.SetInverted(true);
    } else {
        m_baseMotor.SetInverted(false);
    }
    m_baseMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{baseCurrent}});

    double pidTopOutput = m_pidTop.Calculate(dAngTop);
    add = 5;
    double topCurrent = feedForwardTop;//std::clamp(pidTopOutput+feedForwardTop, -m_maxAmpsTop, m_maxAmpsTop);
    if (topCurrent < 0) {
        topCurrent -= add;
    } else if (topCurrent > 0) {
        topCurrent += add;
    }
    //m_topMotor.SetVoltage(units::volt_t{topVoltage});
    if (topCurrent < 0) {
        m_topMotor.SetInverted(true);
    } else {
        m_topMotor.SetInverted(false);
    }
    m_topMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{topCurrent}});

    frc::SmartDashboard::PutBoolean("Target", true);
    frc::SmartDashboard::PutNumber("Target Distance", distance);
    if(configPID){
        m_angOffsetBase = frc::SmartDashboard::GetNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::GetNumber("Top Ang Offset", m_angOffsetTop);
    }
    if (debug) {
        // frc::SmartDashboard::PutNumber("Target X", m_targetX);
        // frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
        frc::SmartDashboard::PutNumber("Base Current", baseCurrent);
        frc::SmartDashboard::PutNumber("Top Current", topCurrent);
    }
}

// Turns the motors off
void Arm::DisabledInit() {
    //TODO: figure out how to set neutral mode
    //m_baseMotor.SetNeutralMode(NeutralMode::Brake);
    //m_topMotor.SetNeutralMode(NeutralMode::Brake);
    m_topMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{0}});
    m_baseMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{0}});
}

void Arm::DisabledPeriodic(){
    if(configPID){
        m_angOffsetBase = frc::SmartDashboard::GetNumber("Base Ang Offset", m_angOffsetBase);
        m_angOffsetTop = frc::SmartDashboard::GetNumber("Top Ang Offset", m_angOffsetTop);
    }
    //frc::SmartDashboard::PutNumber("Ticks Bottom", m_baseMotor.GetSelectedSensorPosition());
    //frc::SmartDashboard::PutNumber("Ticks Top", m_topMotor.GetSelectedSensorPosition());
}

void Arm::TestPeriodic(double ampBase, double ampTop){
    if(configPID){
        frc::SmartDashboard::PutNumber("Base Ang Offset", m_angOffsetBase);
        frc::SmartDashboard::PutNumber("Top Ang Offset", m_angOffsetTop);
    }
    m_baseMotor2.SetControl(controls::Follower(m_baseMotor.GetDeviceID(), false));
    m_topMotor2.SetControl(controls::Follower(m_topMotor.GetDeviceID(), false));

    m_baseMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{ampBase}});
    m_topMotor.SetControl(controls::TorqueCurrentFOC{units::ampere_t{ampTop}});

    if(debug){
        frc::SmartDashboard::PutNumber("Base Amps", ampBase);
        frc::SmartDashboard::PutNumber("Top Amps", ampTop);
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
        m_maxAmpsTop = frc::SmartDashboard::GetNumber("Max Amps Top", m_maxAmpsTop);
        m_maxAmpsBot = frc::SmartDashboard::GetNumber("Max Amps Bot", m_maxAmpsBot);
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
    frc::SmartDashboard::PutNumber("Target X", m_targetX);
    frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
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
    frc::SmartDashboard::PutNumber("Target X", m_targetX);
    frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
    
}

/**
 * @brief Ideally will be called by driverstation to reset the target, for arm positioning purposes. Also resets PID to recalculate for new values.
 */
void Arm::resetTarget() {
    m_targetX = 0.0;
    m_targetZ = m_baseArmLength + m_topArmLength + m_pivotHeight;
    m_pidBase.Reset();
    m_pidTop.Reset();
    frc::SmartDashboard::PutNumber("Target X", m_targetX);
    frc::SmartDashboard::PutNumber("Target Z", m_targetZ);
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