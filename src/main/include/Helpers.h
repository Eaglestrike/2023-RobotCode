#include <cmath>

#include <units/angle.h>

namespace Helpers
{
    void normalizeAngle(double& angle);
    units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution);
}