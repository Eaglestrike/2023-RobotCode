#include <ctre/Phoenix.h>
#include "Constants.h"

namespace Helpers{
    /**
     * 
    */
    static double getPrincipalAng(double ang){//Input radians, output [0, 2*pi]
        int div = ang / (2.0*M_PI);
        double multiple = ((double)div) * 2.0 * M_PI;
        double mod = ang - multiple; 
        return mod;
    }

/**
 * 
*/
    static double getPrincipalAng2(double ang){//Input radians, output [-pi, pi]
        double div = floor(((ang - M_PI) / (2.0*M_PI)) + 1);
        double multiple = div * 2.0*M_PI;
        double mod = ang - multiple; 
        return mod;
    }

/**
 * 
*/
    static double getAbsAng(WPI_TalonFX& motor){//Radians
        double ticks = motor.GetSelectedSensorPosition();
        return ticks/GeneralConstants::TICKS_PER_RADIAN;
    }

    /**
     * 
    */
    static double getAng(WPI_TalonFX& motor){//Returns in range [0, 2*pi]
        double ang = getAbsAng(motor);
        return getPrincipalAng(ang);
    }

    /**
     * 
    */
    static double getAngDiff(double ang1, double ang2){//Angle from ang1 to ang2
        return getPrincipalAng2(ang2 - ang1);
    }

    static double getAbsAngDiff(double ang1, double ang2){//Just subtract lmao
        return ang2 - ang1;
    }
}