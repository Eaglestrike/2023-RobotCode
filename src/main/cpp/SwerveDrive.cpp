#include "SwerveDrive.h"

/**
 * Initializes a swerve drive object (for right now that just entails getting navx + limelight & setting some PID settings)
 * @param nx the navx object
 * @param limelight the limelight object
**/
SwerveDrive::SwerveDrive(AHRS * nx, Limelight limelight): navx_{nx}, limelight_{limelight}
{
    angPID_.EnableContinuousInput(-180, 180);
    angPID_.SetIntegratorRange(-0.5, 0.5);
    initializeOdometry(frc::Rotation2d{0_deg}, frc::Pose2d{frc::Translation2d{0_m, 0_m}, frc::Rotation2d{0_deg}});
}

void SwerveDrive::configSpeedPID(double P, double I, double D) {
    angPID_.SetP(P);
    angPID_.SetI(I);
    angPID_.SetD(D);
}

/**
 * Set's swerve drive's state
**/
void SwerveDrive::setState(State state) {
    state_ = state;
}

/**
 * Gets swerve drive's state
**/
SwerveDrive::State SwerveDrive::getState() {
    return state_;
} 

/**
 * Initializes the odometry object with an initial angle and 2d position
 * @param gyroAngle the robot's initial heading/angle
 * @param initPose the robots initial position on the field
**/
void SwerveDrive::initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose) {
    // delete odometry_;
    odometry_ = new frc::SwerveDriveOdometry<4>(kinematics_, gyroAngle, initPose);
}

/**
 * Updates or resets the odometry positin to the given position and orientation
**/
void SwerveDrive::updateOdometry(frc::Rotation2d robotAngle, frc::Pose2d robotPose) {
    odometry_->ResetPosition(robotPose, robotAngle);
}

/**
 * Called in autonomous init to initialize swerve's trajectory and ramsete (trajectory following) command.
**/
void SwerveDrive::initializeAutoTraj(std::string filePath) {
    ramseteCommand_ = setupRamsete(filePath); 
}

/**
 * @returns moduleStates, an array that contains the speed and angle of each swerve module as a SwerveModuleState
**/
wpi::array<frc::SwerveModuleState, 4> SwerveDrive::getRealModuleStates() {
    wpi::array<frc::SwerveModuleState, 4> moduleStates = {
        flModule_.getState(), frModule_.getState(), blModule_.getState(), brModule_.getState() };
    return moduleStates;
}

/**
 * @returns a DifferentialDriveWheelSpeeds object representing the swerve drive's left side & right side wheel speeds. 
 * For use in trajectory following where swerve drive is treated as differential drive
**/
frc::DifferentialDriveWheelSpeeds SwerveDrive::getDifferentialWheelSpeeds() {
    auto moduleStates = getRealModuleStates();
    double leftSpeed = (moduleStates[0].speed() + moduleStates[2].speed())/2;
    double rightSpeed = (moduleStates[1].speed() + moduleStates[3].speed())/2;
    return {units::meters_per_second_t(leftSpeed), units::meters_per_second_t(rightSpeed)};
}

/**
 * Called periodically in Robot.cpp. encapsulates driving, path following, and odometry update
 * @param dx the desired robot velocity in the x direction
 * @param dy the desired robot velocity in the y direction
 * @param dtheta the desired robot rotational velocity
 * @param turretAngle the turret's angle on the robot, used to help calculate odometry
**/
void SwerveDrive::Periodic(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, double turretAngle) {

    //update odometry no matter the state
    odometry_->Update(units::degree_t{navx_->GetYaw()}, getRealModuleStates());
    lPose_ = limelight_.getPose(navx_->GetYaw(), turretAngle);

    /** Limelight error checking
     * Only update if limelight can see the target
     * If the reported X or Y positions are > 10, it's probably an error and odometry shouldn't be updated
     * If it's been too long (35ms right now) since the last vision update, likely the data is out of date & odometry shouldn't be updated
    */
    if (limelight_.hasTarget() && 
        !(abs(lPose_.X().value()) > 10 
        || abs(lPose_.Y().value()) > 10 
        || (frc::Timer::GetFPGATimestamp().value() - limelight_.getLastUpdated()) >= 35)) {
        updateLimelightOdom(turretAngle, false); 
    }

    //takes appropriate action based on current state
    switch(state_) {
        case DRIVE:
            drive(dx, dy, dtheta, turretAngle);
            break;
        case PATH_FOLLOW:
            ramseteCommand_->Execute();
            odometry_->Update(units::degree_t{navx_->GetYaw()}, getRealModuleStates());
            break;
        case STOP:
            stop();
            break;
    }

}

/**
 * Controls swerve modules to drive in response to joystick driver input
 * @param dx the desired robot velocity in the x direction
 * @param dy the desired robot velocity in the y direction
 * @param dtheta the desired robot rotational velocity
 * @param turretAngle the turret's angle on the robot, used to help calculate odometry
**/
void SwerveDrive::drive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, double turretAngle) {
    units::degree_t navx_yaw = units::degree_t{navx_->GetYaw()};
  
    //converts field-relative joystick input to robot-relative speeds
    frc::ChassisSpeeds speeds_ = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        dx, dy, dtheta, frc::Rotation2d(navx_yaw));

    //convert robot speeds into swerve module states (speed & angle of each module)
    auto [fl, fr, bl, br] = kinematics_.ToSwerveModuleStates(speeds_);

    //optimize module states so they can have most direct path to goal speed + angle
    auto fl_opt = flModule_.getOptState(fl);
    auto fr_opt = frModule_.getOptState(fr);
    auto bl_opt = blModule_.getOptState(bl);
    auto br_opt = brModule_.getOptState(br);

    //set motor outputs
    //TODO: tune drive pid
    flModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(flModule_.getYaw(), fl_opt.angle.Degrees().value()),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

    flModule_.setSpeedMotor( 0.2*std::clamp(fl_opt.speed.value(), -1.0, 1.0) );

    frModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(frModule_.getYaw(), fr_opt.angle.Degrees().value()),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

    frModule_.setSpeedMotor( 0.2*std::clamp(fr_opt.speed.value(), -1.0, 1.0) );

    blModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(blModule_.getYaw(), bl_opt.angle.Degrees().value()),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

    blModule_.setSpeedMotor( 0.2*std::clamp(bl_opt.speed.value(), -1.0, 1.0) );

    brModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(brModule_.getYaw(), br_opt.angle.Degrees().value()),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );

    brModule_.setSpeedMotor( 0.2*std::clamp(br_opt.speed.value(), -1.0, 1.0) );

}

/**
 * Controls swerve modules to behave like differential drive with given @param leftVolts and @param rightVolts
**/
void SwerveDrive::differentialDrive(units::volt_t leftVolts, units::volt_t rightVolts) {
    //voltage constraints
    leftVolts = std::clamp(leftVolts, units::volt_t{-GeneralConstants::MAX_VOLTAGE}, units::volt_t{-GeneralConstants::MAX_VOLTAGE});
    rightVolts = std::clamp(rightVolts, units::volt_t{-GeneralConstants::MAX_VOLTAGE}, units::volt_t{-GeneralConstants::MAX_VOLTAGE});

    flModule_.setSpeedMotorVolts(leftVolts);
    blModule_.setSpeedMotorVolts(leftVolts);

    frModule_.setSpeedMotorVolts(rightVolts);
    brModule_.setSpeedMotorVolts(rightVolts);

    //set all angles to 0 because this is differential drive
    flModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(flModule_.getYaw(), 0),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );
    frModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(frModule_.getYaw(), 0),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );
    blModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(blModule_.getYaw(), 0),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );
    brModule_.setAngMotorVoltage( std::clamp(
        angPID_.Calculate(brModule_.getYaw(), 0),
        -GeneralConstants::MAX_VOLTAGE, GeneralConstants::MAX_VOLTAGE) );
}

/**
 * Freezes all swerve modules in place such that the robot stops moving.
**/
void SwerveDrive::stop() {
    flModule_.setAngMotorVoltage(0);
    flModule_.setSpeedMotor(0);

    frModule_.setAngMotorVoltage(0);
    frModule_.setSpeedMotor(0);

    blModule_.setAngMotorVoltage(0);
    blModule_.setSpeedMotor(0);

    brModule_.setAngMotorVoltage(0);
    brModule_.setSpeedMotor(0);
}

/**
 * @param filePath the name of the file to be used for trajectory generation
 * @returns a ramseteCommand shared pointer object that, when executed, will result in the robot following the inputted traj file
 * Using a shared pointer so that the ramsete command can be initialized after construction, and so that the pointer can
 * change owners (from this function to global SwerveDrive class)
**/
std::shared_ptr<frc2::RamseteCommand> SwerveDrive::setupRamsete(std::string filePath) {
    frc::TrajectoryConfig config{SwerveConstants::kMaxSpeed, SwerveConstants::kMaxAccel};
    config.SetKinematics(kinematics_);

    //get trajectory from pathweaver file
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    deployDirectory = deployDirectory / "paths" / filePath;
    frc::Trajectory trajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());

    updateOdometry(trajectory.InitialPose().Rotation(), trajectory.InitialPose());
    navx_->SetAngleAdjustment(-trajectory.InitialPose().Rotation().Degrees().value());

    std::shared_ptr<frc2::RamseteCommand> ramseteCommand = make_shared<frc2::RamseteCommand>(
        frc2::RamseteCommand(
        trajectory, [this]() { return odometry_->GetPose(); }, //lambda function to get robot's position
        frc::RamseteController(SwerveConstants::kRamseteB, SwerveConstants::kRamseteZeta), 
        frc::SimpleMotorFeedforward<units::meters>(SwerveConstants::ks, SwerveConstants::kv, SwerveConstants::ka),
        diffDriveKinematics_, 
        [this] { return getDifferentialWheelSpeeds(); }, //lambda function to get the robot's current wheel speeds
        speedPID_, speedPID_,
        [this] (units::volt_t leftVolts, units::volt_t rightVolts) { differentialDrive(leftVolts, rightVolts); } //lambda function to set wheel volts
    ));
    return ramseteCommand;
}


/**
 * @returns the robot's current velocity 
**/
frc::ChassisSpeeds SwerveDrive::getRobotSpeeds() {
  frc::ChassisSpeeds speeds = kinematics_.ToChassisSpeeds(getRealModuleStates());
  //rest of stuff would be for conversion to field-relative speeds, alone speeds is robot relative
//   frc::Translation2d t{units::meter_t{speeds.vx.value()}, units::meter_t{speeds.vy.value()}}; //sus units... if we hate it i can write this myself
//   t.RotateBy(frc::Rotation2d{units::degree_t{m_navx->GetYaw()}});
//   speeds.vx = units::meters_per_second_t{t.X().value()};
//   speeds.vy = units::meters_per_second_t{t.Y().value()};
  return speeds;
}

/**
 * @returns the distance from the center of the robot to the center of the goal
**/
double SwerveDrive::getDistance(double turretAngle)
{
    std::pair<double, double> limelightToRobot = camToBot(turretAngle);  //vector from limelight pose to center of bot
    
    double limelightToGoalX = odometry_->GetPose().X().value() + limelightToRobot.first;
    double limelightTtoGoalY = odometry_->GetPose().Y().value() + limelightToRobot.second;

    return sqrt(limelightToGoalX * limelightToGoalX + limelightTtoGoalY * limelightTtoGoalY);
}

/**
 * Compute vector from limelight position to center of robot
**/
std::pair<double, double> SwerveDrive::camToBot(double turretAngle) {

    //normalize turret angle
    double turretLimelightAngle = turretAngle - 180;
    frc::InputModulus(turretLimelightAngle, -180.0, 180.0);
    turretLimelightAngle = turretLimelightAngle * M_PI / 180;

    //get x and y component (robot relative) of vector from center of turret to limelight
    double turretLimelightX = LimelightConstants::TURRET_CENTER_RADIUS * sin(turretLimelightAngle);
    double turretLimelightY = LimelightConstants::TURRET_CENTER_RADIUS * cos(turretLimelightAngle);

    //subtract(?) constant distance from center of robot to center of turret
    turretLimelightY -= LimelightConstants::ROBOT_TURRET_CENTER_DISTANCE;

    //rotate this vector by the robot's angle to make it field relative
    //navx angle is clockwise positive & this rotation is clockwise negative, so angle can be positive
    double angle = navx_->GetYaw();
    double robotLimelightX = turretLimelightX * cos(angle) - turretLimelightY * sin(angle);
    double robotLimelightY = turretLimelightX * sin(angle) + turretLimelightY * cos(angle);

    return {robotLimelightX, robotLimelightY};    
}


/**
 * Average wheel odometry with limelight calculated position
**/
void SwerveDrive::updateLimelightOdom(double turretAngle, bool inAuto)
{
    //get limelight pose (so this is the position of the camera)
    double limelightX_ = lPose_.X().value();
    double limelightY_ = lPose_.Y().value();

    //adjust to get position of actual robot
    std::pair<double, double> trans = camToBot(turretAngle);

    limelightX_ -= trans.first;
    limelightY_ -= trans.second;

    units::meter_t averagedX = units::meter_t{0.04*limelightX_ + 0.96*odometry_->GetPose().X().value()};
    units::meter_t averagedY = units::meter_t{0.04*limelightY_ + 0.96*odometry_->GetPose().Y().value()};
    frc::Rotation2d rot = frc::Rotation2d{units::degree_t{navx_->GetYaw()}};

    updateOdometry(rot, frc::Pose2d{averagedX, averagedY, rot});
}

/**
 * @returns robot-relative angle to goal
 * Goal is (0, 0). X and Y technically negative relative to goal, but atan so doesn't really matter
**/
double SwerveDrive::getRobotGoalAng(double turretAngle)
{
    return atan2(-getY(), -getX());
}


double SwerveDrive::getX()
{
    return odometry_->GetPose().X().value();
}

double SwerveDrive::getY()
{
    return odometry_->GetPose().Y().value();
}
