#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "AprilTags.h"


class AprilTagPoseEstimator {
public:
    /**
     * Returns the robot position based on an april tag position reading.
     * @param aprilTagPosReading april tag position reading @see AprilTagPoseEstimator.cpp for more details about this argument
     * @param aprilTagNum april tag number
     * @return robot position
    */
    frc::Pose2d getPose(frc::Pose3d aprilTagPosReading, int aprilTagNum);

private:
    frc::AprilTagFieldLayout aprilTagFieldLayout = AprilTags::getAprilTagFieldLayout();
};