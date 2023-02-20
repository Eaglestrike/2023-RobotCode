#include <math.h>

#include <units/angle.h>
#include "Constants.h"

namespace Helpers
{
    void normalizeAngle(double& angle);
    //units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution);

    double getPrincipalAng2(double ang);
    double getPrincipalAng2Deg(double ang);
}