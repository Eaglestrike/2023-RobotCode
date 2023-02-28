#include "Helpers.h"

namespace Helpers {

    /**
     * modifies the angle to be in range [-180, 180]
    */
    void normalizeAngle(double& angle)
    {
        angle += 360 * 10;
        angle = ((int) floor(angle) % 360) + (angle - floor(angle));
        angle -= 360 * floor(angle / 360 + 0.5);
    }

// units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution) {
//   double valRad = val * (2 * pi / numStepsPerRevolution);
//   return units::radian_t{valRad};
// }
} // namespace Helpers