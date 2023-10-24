#include "DrivePoseEstimator.h"

// this should be called periodically, as often as possible
void DrivePoseEstimator::updateWheels() {
    // update using wheel odometry:
    // estimator.Update(navx->GetRotation2d(), swerveDrive->GetRealModuleStates());
}

// this should be called whenever the camera gets an apriltag reading
void DrivePoseEstimator::updateCamera() {
    // pose = get robot pose from vision
    // timestamp = get image timestamp, or just current timestamp using getFPGATimestamp if that's not avilable
    // stddevs = calculate based on the distance from the apriltag
    // estimator.AddVisionMeasurement(pose, timestamp, stddevs)
}

frc::Pose2d DrivePoseEstimator::getPose() {
    
}