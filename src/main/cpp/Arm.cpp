#include "Arm.h"

void Arm::periodic(){
    if(m_targetZ < 0.0){//If target is under the floor
        return;
    }
    double targetdy = m_pivotHeight - m_targetZ;
    double distance = sqrt((m_targetX*m_targetX) + (targetdy*targetdy));
    if(distance > m_baseArmLength + m_topArmLength){
        return;
    }
    //Finding ideal angles
    //https://www.google.com/search?q=law+of+cosine
    double a = m_baseArmLength;
    double b = m_topArmLength;
    double c = distance;
    double baseArmAng = acos(((c*c)-(a*a)-(b*b))/(2*a*b)); //Angle between 2 arms
    //https://www.google.com/search?q=law+of+sines
    double topArmAng = (sin(baseArmAng)/c) * b; //Angle of base arm
    
}

void Arm::setTarget(double targetX, double targetZ){
    m_targetX = targetX;
    m_targetZ = targetZ;
}