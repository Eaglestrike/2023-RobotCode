#include "Helpers/GeometryHelper.h"

double GeometryHelper::toDeg(double rad){
    return rad / M_PI * 180.0;
}

double GeometryHelper::toRad(double deg){
    return deg * M_PI / 180.0;
}

/**
 * Input radians, output angle in range [0, 2*pi]
*/
double GeometryHelper::getPrincipalAng(double ang){
    int div = ang / (2.0*M_PI);
    double multiple = ((double)div) * 2.0 * M_PI;
    double mod = ang - multiple; 
    return mod;
}

/**
 * @param ang Input radians
 * @returns output in range [-pi, pi]
*/
double GeometryHelper::getPrincipalAng2(double ang){
    //https://www.desmos.com/calculator/yctu2qoevr
    double div = floor(((ang - M_PI) / (2.0*M_PI)) + 1);
    double multiple = div * 2.0*M_PI;
    double mod = ang - multiple; 
    return mod;
}

/**
 * @param ang Input degrees
 * @returns output in range [-180, 180]
*/
double GeometryHelper::getPrincipalAng2Deg(double ang){
    double div = floor(((ang - 180.0)/ (360.0)) + 1);
    double multiple = div * 360.0;
    double mod = ang - multiple; 
    return mod;
}

/// @brief interior Angle from ang1 to ang2
/// @param ang1 radians
/// @param ang2 radians
/// @return radians
double GeometryHelper::getAngDiff(double ang1, double ang2){
    return getPrincipalAng2(ang2 - ang1);
}

/// @brief interior Angle from ang1 to ang2
/// @param ang1 degrees
/// @param ang2 degrees
/// @return degrees
double GeometryHelper::getAngDiffDeg(double ang1, double ang2){
    return getPrincipalAng2Deg(ang2 - ang1);
}

/**
* @param angMid angle tested in between 1 and 2
*/
bool GeometryHelper::angInBetween(double angMid, double ang1, double ang2){
    //https://www.desmos.com/calculator/bo2iuiuukb
    double angDiff12 = getAngDiff(ang1, ang2);
    double angDiff1Mid = getAngDiff(ang1, angMid);
    //std::cout<< "angMid: " << angMid << ", ang1: " << ang1 << ", ang2: " << ang2 << std::endl;
    //std::cout<< "ang diffl2:" <<angDiff12<< ", angDiff1Mid:" << angDiff1Mid << std::endl;
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

double GeometryHelper::getAbsAngDiff(double ang1, double ang2){//Just subtract lmao
    return ang2 - ang1;
}