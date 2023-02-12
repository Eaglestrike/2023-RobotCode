#include "Helpers.h"

namespace Helpers {
void normalizeAngle(double& angle)
{
    angle += 360 * 10;
    angle = ((int) std::floor(angle) % 360) + (angle - std::floor(angle));
    angle -= 360 * std::floor(angle / 360 + 0.5);
}

units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution) {
  double valRad = val * (2 * M_PI / numStepsPerRevolution);
  return units::radian_t{valRad};
}
} // namespace Helpers