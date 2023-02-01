#include "AprilTagPoseEstimator.h"
#include <cmath>

#include <iostream>

using namespace frc;

// this function expects to get the april tag the april tag pose reading in the following format:
// x - 1st value from jetson
// y - 2nd value from jetson
// z - 3rd value from jetson
// roll (x, 1st value in rotation3d) - 4th value from jetson
// pitch (y, 2nd value in rotation3d) - 5th value from jetson
// yaw (z, 3rd value in rotation3d) - 6th value from jetson
// aprilTagNum: april tag number
Pose2d AprilTagPoseEstimator::getPose(Pose3d aprilTagPosReading, int aprilTagNum) {
    // reverse signs, because we want to get the robot position relative to the
    // apriltag, NOT the apriltag position relative to the robot
    units::meter_t x = -aprilTagPosReading.X();
    units::meter_t y = -aprilTagPosReading.Y();
    units::radian_t gamma = -aprilTagPosReading.Rotation().Z();

    Pose3d aprilTagPos = aprilTagFieldLayout.GetTagPose(aprilTagNum).value();

    std::cout << aprilTagFieldLayout.GetTagPose(0).value().X().value() << "\n";
    std::cout << aprilTagFieldLayout.GetTagPose(0).value().Y().value() << "\n";
    std::cout << aprilTagFieldLayout.GetTagPose(0).value().Z().value() << "\n";
    std::cout << "\n";

    // apriltag rotation
    units::radian_t aprilTagRot = aprilTagPos.Rotation().ToRotation2d().Radians();

    // calculate rotation sine and cosine values
    units::radian_t rotAngle = aprilTagRot - Rotation2d(90_deg).Radians() + gamma;
    double cosVal = std::cos(unit_cast<double>(rotAngle));
    double sinVal = std::sin(unit_cast<double>(rotAngle));

    // calculated translated x and y values (robot pose relative to apriltag with correct coordinates)
    units::meter_t translatedX = x * cosVal - y * sinVal;
    units::meter_t translatedY = x * sinVal + y * cosVal;

    // add april tag location coordinates to get robot position
    units::meter_t robotX = translatedX + aprilTagPos.X();
    units::meter_t robotY = translatedY + aprilTagPos.Y();
    units::radian_t robotRot = aprilTagRot + gamma;

    return Pose2d{robotX, robotY, robotRot};
}



frc::AprilTagFieldLayout AprilTagPoseEstimator::updateField(Pose3d newPose, int tagNum) {
    AprilTag tag(tagNum, newPose);
    std::vector<AprilTag> aprilTags = {tag};
    
    aprilTagFieldLayout = AprilTagFieldLayout(aprilTags, AprilTagsConstants::FIELD_LENGTH, AprilTagsConstants::FIELD_WIDTH);

    return aprilTagFieldLayout;
}
