#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "AprilTags.h"


class AprilTagPoseEstimator {
public:
    frc::Pose2d getPose(frc::Pose3d aprilTagPos, int aprilTagNum);

private:
    frc::AprilTagFieldLayout aprilTagFieldLayout = AprilTags::getAprilTagFieldLayout();
};