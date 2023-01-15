#include "AprilTags.h"
#include <frc/apriltag/AprilTag.h>
#include <units/length.h>
#include <frc/DriverStation.h>
#include "Constants.h"


using namespace units::length;
using namespace frc;

frc::AprilTagFieldLayout AprilTags::getAprilTagFieldLayout() {
    // if blue alliance or invalid
    if (DriverStation::GetAlliance() != DriverStation::Alliance::kRed) {
        AprilTag tag1(1, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_LOW_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag2(2, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_MID_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag3(3, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_TOP_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag4(4, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_X,
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag5(5, Pose3d(Pose2d(
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_X,
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag6(6, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_TOP_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag7(7, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_MID_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag8(8, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_LOW_Y,
            Rotation2d((units::degree_t) 0)
        )));
        std::vector<AprilTag> aprilTags;
        aprilTags.push_back(tag1);
        aprilTags.push_back(tag2);
        aprilTags.push_back(tag3);
        aprilTags.push_back(tag4);
        aprilTags.push_back(tag5);
        aprilTags.push_back(tag6);
        aprilTags.push_back(tag7);
        aprilTags.push_back(tag8);
        AprilTagFieldLayout layout(
            aprilTags,
            AprilTagsConstants::FIELD_LENGTH, AprilTagsConstants::FIELD_WIDTH
        );
        return layout;
    } else { // if red alliance
        AprilTag tag1(1, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_LOW_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag2(2, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_MID_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag3(3, Pose3d(Pose2d(
            AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_TOP_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag4(4, Pose3d(Pose2d(
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_X,
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_Y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag5(5, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_X,
            AprilTagsConstants::DOUBLE_SUBSTATION_APRILTAG_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag6(6, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_TOP_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag7(7, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_MID_Y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag8(8, Pose3d(Pose2d(
            AprilTagsConstants::FIELD_LENGTH - AprilTagsConstants::GRID_APRILTAG_X,
            AprilTagsConstants::GRID_APRILTAG_LOW_Y,
            Rotation2d((units::degree_t) 180)
        )));
        std::vector<AprilTag> aprilTags;
        aprilTags.push_back(tag1);
        aprilTags.push_back(tag2);
        aprilTags.push_back(tag3);
        aprilTags.push_back(tag4);
        aprilTags.push_back(tag5);
        aprilTags.push_back(tag6);
        aprilTags.push_back(tag7);
        aprilTags.push_back(tag8);
        AprilTagFieldLayout layout(
            aprilTags,
            AprilTagsConstants::FIELD_LENGTH, AprilTagsConstants::FIELD_WIDTH
        );
        return layout;
    }
}