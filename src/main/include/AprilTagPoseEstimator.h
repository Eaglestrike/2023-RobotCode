#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include "AprilTags.h"
#include "Constants.h"


class AprilTagPoseEstimator {
public:
    /**
     * Returns the robot position based on an april tag position reading.
     * @param aprilTagPosReading april tag position reading @see AprilTagPoseEstimator.cpp for more details about this argument
     * @param aprilTagNum april tag number
     * @return robot position
    */
    frc::Pose2d getPose(frc::Pose3d aprilTagPosReading, int aprilTagNum);

    void updatePeriodic();

    //for testing purposes
    frc::AprilTagFieldLayout updateField(frc::Pose3d newPose, int tagNum);

private:
    frc::AprilTagFieldLayout aprilTagFieldLayout = AprilTags::getAprilTagFieldLayout();
  
};