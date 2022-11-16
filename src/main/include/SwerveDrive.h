#pragma once

#include <ctre/Phoenix.h>
#include <iostream>
#include <math.h>
#include "Controls.h"
#include "Constants.h"
#include "Limelight.h"
#include <AHRS.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include "SwerveModule.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <algorithm>
#include <memory>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>


class SwerveDrive
{
    public:
        
        enum State{
            DRIVE,
            PATH_FOLLOW,
            STOP
        };
        void setState(State state);
        State getState();

        SwerveDrive(AHRS * nx, Limelight limelight); //todo: add logger
        void setYaw(double yaw);
        
        void Periodic(units::meters_per_second_t joy_x, units::meters_per_second_t joy_y, 
        units::radians_per_second_t joy_theta, double turretAngle);

        void initializeOdometry(frc::Rotation2d gyroAngle, frc::Pose2d initPose);
        void updateOdometry(frc::Rotation2d robotAngle, frc::Pose2d robotPose);
       
        void initializeAutoTraj(std::string filePath);

        wpi::array<frc::SwerveModuleState, 4> getRealModuleStates(); //real as opposed to goal

        void updateLimelightOdom(double turretAngle, bool inAuto);

        double getDistance(double turretAngle);
        frc::ChassisSpeeds getRobotSpeeds();
        double getX();
        double getY();
        double getRobotGoalAng(double turretAngle);

        void configSpeedPID(double P, double I, double D);

    private:
        AHRS * navx_;
        Limelight limelight_;

        frc::SwerveDriveOdometry<4> * odometry_; //will need to be initialized later with selected robot start pose
        frc::Pose2d lPose_; //public so limelight calculation will only happen once per period to save time
        State state_;

        std::pair<double, double> camToBot(double turretAngle);
        void drive(units::meters_per_second_t dx, units::meters_per_second_t dy, units::radians_per_second_t dtheta, double turretAngle);
        void stop();

        frc::DifferentialDriveWheelSpeeds getDifferentialWheelSpeeds();
        void differentialDrive(units::volt_t leftVolts, units::volt_t rightVolts);
        std::shared_ptr<frc2::RamseteCommand> setupRamsete(std::string filePath);
        std::shared_ptr<frc2::RamseteCommand> ramseteCommand_; //will be set up in auto init

        frc::SwerveDriveKinematics<4> kinematics_{
            frc::Translation2d{0.3683_m, 0.3683_m}, frc::Translation2d{0.3683_m, -0.3683_m},
            frc::Translation2d{-0.3683_m, 0.3683_m}, frc::Translation2d{-0.3683_m, -0.3683_m}
        };

        frc::DifferentialDriveKinematics diffDriveKinematics_{0.7366_m};


        SwerveModule flModule_{SwerveConstants::FLanglePort, SwerveConstants::FLspeedPort, SwerveConstants::FLencoder, SwerveConstants::FLOFF, false};
        SwerveModule frModule_{SwerveConstants::FRanglePort, SwerveConstants::FRspeedPort, SwerveConstants::FRencoder, SwerveConstants::FROFF, false};
        SwerveModule blModule_{SwerveConstants::BLanglePort, SwerveConstants::BLspeedPort, SwerveConstants::BLencoder, SwerveConstants::BLOFF, false};
        SwerveModule brModule_{SwerveConstants::BRanglePort, SwerveConstants::BRspeedPort, SwerveConstants::BRencoder, SwerveConstants::BROFF, false};

        frc2::PIDController angPID_{SwerveConstants::P , SwerveConstants::I, SwerveConstants::D};
        frc2::PIDController speedPID_{SwerveConstants::sP , SwerveConstants::sI, SwerveConstants::sD};    

};
