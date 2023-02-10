#include <ctre/Phoenix.h>
#include "Constants.h"
#include <cmath>
#include <iostream>

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
    static double getAbsAng(WPI_TalonFX& motor, double ratio){//Radians
        double ticks = motor.GetSelectedSensorPosition()/ratio;
        return ticks/GeneralConstants::TICKS_PER_RADIAN;
    }

    /**
     * 
    */
    static double getAng(WPI_TalonFX& motor, double ratio){//Returns in range [0, 2*pi]
        double ang = getAbsAng(motor, ratio);
        return getPrincipalAng(ang);
    }

    /**
     * 
    */
    static double getAngDiff(double ang1, double ang2){//Angle from ang1 to ang2
        return getPrincipalAng2(ang2 - ang1);
    }

    /*
    @param angMid angle tested in between 1 and 2
    https://www.desmos.com/calculator/bo2iuiuukb
    */
    static bool angInBetween(double angMid, double ang1, double ang2){
        double angDiff12 = getAngDiff(ang1, ang2);
        double angDiff1Mid = getAngDiff(ang1, angMid);
       // std::cout<< "ang diffl2:" <<angDiff<< ", top reading:" << topReading << ", ang reading:" <<ang2 << std::endl;
        if(angDiff12 > 0){
            if(angDiff1Mid > 0 && angDiff1Mid < angDiff12){
                return true;
            }
            else{
                return false;
            }
        }
        else{
            if(angDiff1Mid < 0 && angDiff1Mid > angDiff12){
                return true;
            }
            else{
                return false;
            }
        }
    }

    static double getAbsAngDiff(double ang1, double ang2){//Just subtract lmao
        return ang2 - ang1;
    }

    static double scaleError(double k1, double k2, double error){//https://www.desmos.com/calculator/478yleqngu
        double exp = -(1.0/k1)*error*error;
        double hump = k2*std::pow(2.0, exp) + 1.0; 
        return error * hump;
    }
}