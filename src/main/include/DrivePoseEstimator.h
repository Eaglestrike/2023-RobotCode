#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/MathUtil.h>
#include <frc/geometry/Pose2d.h>
#include "Constants.h"



class DrivePoseEstimator {
public:
  void updateWheels();
  void updateCamera();
  frc::Pose2d getPose();

private:

  // TODO tune these hyperparameters
  // these hyperparameters specify how accurate each measurement is (how much to trust it)
  const wpi::array<double, 3> stateStdDevs = {0.01, 0.01, 0.01};
  const wpi::array<double, 1> localMeasurementStdDevs = {0.1};
  const wpi::array<double, 3> visionMeasurementStdDevs = {0.1, 0.1, 0.1};

  // This is the kalman filter that combines vision and wheel odometry measurements
  //frc::SwerveDrivePoseEstimator<4> estimator 
};