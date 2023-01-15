#include "AprilTags.h"
#include <frc/apriltag/AprilTag.h>
#include <units/length.h>
#include <frc/DriverStation.h>


using namespace units::length;
using namespace frc;

frc::AprilTagFieldLayout AprilTags::getAprilTagFieldLayout() {
    // check if we are blue alliance
    units::meter_t field_length = 54_ft + 3.25_in;
    units::meter_t field_width = 26_ft + 3.25_in;
    units::meter_t grid_apriltag_x = 3_ft + 1_in;
    units::meter_t grid_apriltag_low_y = 3_ft + 6_in;
    units::meter_t grid_apriltag_mid_y = 9_ft;
    units::meter_t grid_apriltag_top_y = 14_ft + 6_in;
    units::meter_t double_substation_apriltag_x = 1_ft + 2_in;
    units::meter_t double_substation_apriltag_y = 22_ft + 0.3125_in;
    // if blue alliance or invalid
    if (DriverStation::GetAlliance() != DriverStation::Alliance::kRed) {
        AprilTag tag1(1, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_low_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag2(2, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_mid_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag3(3, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_top_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag4(4, Pose3d(Pose2d(
            field_length - double_substation_apriltag_x,
            double_substation_apriltag_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag5(5, Pose3d(Pose2d(
            double_substation_apriltag_x,
            double_substation_apriltag_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag6(6, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_top_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag7(7, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_mid_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag8(8, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_low_y,
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
            field_length, field_width
        );
        return layout;
    } else { // if red alliance
        AprilTag tag1(1, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_low_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag2(2, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_mid_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag3(3, Pose3d(Pose2d(
            grid_apriltag_x,
            grid_apriltag_top_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag4(4, Pose3d(Pose2d(
            double_substation_apriltag_x,
            double_substation_apriltag_y,
            Rotation2d((units::degree_t) 0)
        )));
        AprilTag tag5(5, Pose3d(Pose2d(
            field_length - double_substation_apriltag_x,
            double_substation_apriltag_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag6(6, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_top_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag7(7, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_mid_y,
            Rotation2d((units::degree_t) 180)
        )));
        AprilTag tag8(8, Pose3d(Pose2d(
            field_length - grid_apriltag_x,
            grid_apriltag_low_y,
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
            field_length, field_width
        );
        return layout;
    }
}