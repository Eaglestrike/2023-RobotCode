#include "Helpers.h"

namespace Helpers {
    void normalizeAngle(double& angle)
    {
        angle += 360 * 10;
        angle = ((int) floor(angle) % 360) + (angle - floor(angle));
        angle -= 360 * floor(angle / 360 + 0.5);
    }

    /**
     * @param ang Input radians
     * @returns output in range [-pi, pi]
    */
    double getPrincipalAng2(double ang){
        //https://www.desmos.com/calculator/yctu2qoevr
        double div = floor(((ang - M_PI) / (2.0*M_PI)) + 1);
        double multiple = div * 2.0*M_PI;
        double mod = ang - multiple; 
        return mod;
    }

    double getPrincipalAng2Deg(double ang){
        double div = floor(((ang - 180.0)/ (360.0)) + 1);
        double multiple = div * 360.0;
        double mod = ang - multiple; 
        return mod;
    }

units::radian_t convertStepsToRadians(double val, double numStepsPerRevolution) {
  double valRad = val * (2 * M_PI / numStepsPerRevolution);
  return units::radian_t{valRad};
}
} // namespace Helpers