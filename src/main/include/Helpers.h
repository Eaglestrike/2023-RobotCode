#include <math.h>

#include <units/angle.h>
#include "Constants.h"

namespace Helpers
{
    /**
     * modifies the angle to be in range [-180, 180]
    */
    void normalizeAngle(double& angle);
    //units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution);
}