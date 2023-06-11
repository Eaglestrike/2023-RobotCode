#include "SwervePath.h"

SwervePath::SwervePath(double maxLA, double maxLV, double maxAA, double maxAV) : MAX_LA(maxLA), MAX_LV(maxLV), MAX_AA(maxAA), MAX_AV(maxAV)
{

}
        
void SwervePath::setKLP(double klP)
{
    klP_ = klP;
}
        
void SwervePath::setKLD(double klD)
{
    klD_ = klD;
}
        
void SwervePath::setKAP(double kaP)
{
    kaP_ = kaP;
}

void SwervePath::setKAD(double kaD)
{
    kaD_ = kaD;
}

void SwervePath::setKLV(double klV)
{
    klV_ = klV;
}

void SwervePath::setKLA(double klA)
{
    klA_ = klA;
}

void SwervePath::setKAV(double kaV)
{
    kaV_ = kaV;
}
void SwervePath::setKAA(double kaA)
{
    kaA_ = kaA;
}

void SwervePath::generateTrajectory(bool spline)
{
    if(spline)
    {
        generateSplineTrajectory();
    }
    else
    {
        generateLinearTrajectory();
    }
}

void SwervePath::generateLinearTrajectory()
{
    trajectories_.clear();
    
    for(size_t i = 0; i < points_.size() - 1; ++i)
    {
        SwervePose p1 = points_[i];
        SwervePose p2 = points_[i + 1];

        double dx = p2.getX() - p1.getX();
        double dy = p2.getY() - p1.getY();
        double totalLinearDist = sqrt(dx * dx + dy * dy);
        double yawDist = p2.getYawDist();
        if(yawDist > totalLinearDist)
        {
            yawDist = totalLinearDist;
        }
        double dyaw = p2.getYaw() - p1.getYaw();
        Helpers::normalizeAngle(dyaw);
        int yawDirection = (dyaw > 0) ? 1 : -1;
        dyaw = abs(dyaw);

        double yawAccelTime, yawCruiseTime, yawCruiseDist, yawCruiseVel, 
        linYawAccelTime, linYawCruiseTime, linYawCruiseDist, linYawCruiseVel, 
        linYawDeccelTime, actualYawDist, endVel;
        if(dyaw != 0 && (dx != 0 || dy != 0))
        {
            yawCruiseVel = MAX_AA * 0.5 * sqrt(dyaw / (MAX_AA * 0.5));
            if(yawCruiseVel > MAX_AV * 0.5)
            {
                yawCruiseVel = MAX_AV * 0.5;
                yawAccelTime = MAX_AV / MAX_AA;
                double accelDeccelYaw = MAX_AV * yawAccelTime * 0.5;
                yawCruiseDist = dyaw - accelDeccelYaw;
                yawCruiseTime = yawCruiseDist / (MAX_AV * 0.5);
            }
            else
            {
                yawAccelTime = yawCruiseVel / (MAX_AA * 0.5);
                yawCruiseDist = 0;
                yawCruiseTime = 0;
            }

            double totalYawTime = yawAccelTime * 2 + yawCruiseTime;
            double linMaxAccelTime = MAX_LV / MAX_LA;
            double absoluteMaxDist;
            if(linMaxAccelTime <= totalYawTime)
            {
                absoluteMaxDist = linMaxAccelTime * linMaxAccelTime * 0.25 * MAX_LA + (totalYawTime - linMaxAccelTime) * MAX_LV * 0.5;
            }
            else
            {
                absoluteMaxDist = totalYawTime * totalYawTime * 0.25 * MAX_LA;
            }
            double minEndLinDist;
            if(absoluteMaxDist < yawDist)
            {
                minEndLinDist = totalLinearDist - absoluteMaxDist;
            }
            else
            {
                minEndLinDist = totalLinearDist - yawDist;
            }
            

            double maxEndLinVel = MAX_LA * sqrt(minEndLinDist * 2 / MAX_LA); //Normal acceleration
            
            double linMaxDist;
            bool deccelNeeded, deccelFromMax;
            if(maxEndLinVel < MAX_LV * 0.5)
            {
                deccelNeeded = true;
                double maxDeccelTime = (MAX_LV * 0.5 - maxEndLinVel) / (MAX_LA * 0.5);
                if(maxDeccelTime + linMaxAccelTime <= totalYawTime)
                {
                    deccelFromMax = true;
                    //Doesn't account for lost distance
                    linMaxDist = 0.25 * MAX_LV * linMaxAccelTime + ((maxEndLinVel + MAX_LV * 0.5) / 2) * maxDeccelTime + MAX_LV * 0.5 * (totalYawTime - maxDeccelTime - linMaxAccelTime);
                }
                else if ((maxEndLinVel / (MAX_LA * 0.5)) > totalYawTime)
                {
                    deccelFromMax = false;
                    deccelNeeded = false;
                    //Doesn't account for lost distance
                    linMaxDist = MAX_LA * 0.25 * totalYawTime * totalYawTime;
                }
                else
                {
                    deccelFromMax = false;
                    double ghostEndTime = (maxEndLinVel / (MAX_LA * 0.5));
                    double halfGhostTime = (ghostEndTime + totalYawTime) / 2;
                    double maxVel = halfGhostTime * MAX_LA * 0.5;
                    //Doesn't account for lost distance sad
                    linMaxDist = 0.5 * maxVel * halfGhostTime + (halfGhostTime - ghostEndTime) * ((maxVel + maxEndLinVel) / 2);
                }
            }
            else
            {
                deccelNeeded = false;
                if(totalYawTime > linMaxAccelTime)
                {
                    linMaxDist = 0.25 * MAX_LV * linMaxAccelTime + MAX_LV * 0.5 * (totalYawTime - linMaxAccelTime);
                }
                else
                {
                    linMaxDist = MAX_LA * 0.25 * totalYawTime * totalYawTime;
                }
            }

            if(linMaxDist > yawDist)
            {
                actualYawDist = yawDist;
                if(MAX_LA * MAX_LA * 0.25 * totalYawTime * totalYawTime - MAX_LA * yawDist >= 0)
                {
                    linYawAccelTime = ((-MAX_LA * 0.5 * totalYawTime) + sqrt(MAX_LA * MAX_LA * 0.25 * totalYawTime * totalYawTime - MAX_LA * yawDist)) / (-MAX_LA * 0.5);
                    if(linYawAccelTime < 0)
                    {
                        linYawAccelTime = ((-MAX_LA * 0.5 * totalYawTime) - sqrt(MAX_LA * MAX_LA * 0.25 * totalYawTime * totalYawTime - MAX_LA * yawDist)) / (-MAX_LA * 0.5);
                    }
                    linYawCruiseTime = totalYawTime - linYawAccelTime;
                    linYawCruiseVel = linYawAccelTime * MAX_LA * 0.5;
                    linYawCruiseDist = linYawCruiseVel * linYawCruiseTime;
                    linYawDeccelTime = 0;
                    endVel = linYawCruiseVel;
                }
                else
                {
                    linYawAccelTime = totalYawTime; //COULDO should never get here, make better if it does
                    linYawCruiseTime = 0;
                    linYawCruiseVel = 0;
                    linYawCruiseDist = 0;
                    linYawDeccelTime = 0;
                    endVel = 0;
                    cout << "SwervePath.cpp 160 died" << endl;
                }

                if(deccelNeeded && linYawCruiseVel > maxEndLinVel)
                {
                    double num = (MAX_LA*MAX_LA*0.25*totalYawTime*totalYawTime-4*yawDist*MAX_LA*0.5+2*maxEndLinVel*MAX_LA*0.5*totalYawTime-maxEndLinVel*maxEndLinVel);
                    if(num >= 0)
                    {
                        linYawCruiseVel = (-(2*maxEndLinVel+2*MAX_LA*0.5*totalYawTime)+2*sqrt(num))/-4;
                        if(linYawCruiseVel < 0 || linYawCruiseVel > MAX_LV * 0.5)
                        {
                            linYawCruiseVel = (-(2*maxEndLinVel+2*MAX_LA*0.5*totalYawTime)-2*sqrt(num))/-4;
                        }

                        linYawAccelTime = linYawCruiseVel / (MAX_LA * 0.5);
                        linYawDeccelTime = (linYawCruiseVel - maxEndLinVel) / (MAX_LA * 0.5);
                        linYawCruiseTime = totalYawTime - linYawAccelTime - linYawDeccelTime;
                        linYawCruiseDist = linYawCruiseTime * linYawCruiseVel;
                        endVel = maxEndLinVel;
                        
                    }
                    else
                    {
                        linYawAccelTime = 0; //COULDO should never get here, make better if it does
                        linYawCruiseTime = 0;
                        linYawCruiseVel = 0;
                        linYawCruiseDist = 0;
                        linYawDeccelTime = 0;
                        endVel = 0;
                        cout << "SwervePath.cpp 172 died" << endl;
                    }
                    
                }
            }
            else
            {
                actualYawDist = linMaxDist;
                if(deccelNeeded)
                {
                    endVel = maxEndLinVel;
                    if(deccelFromMax)
                    {
                        linYawAccelTime = linMaxAccelTime;
                        linYawCruiseVel = MAX_LV * 0.5;
                        linYawDeccelTime = ((MAX_LV * 0.5) - maxEndLinVel) / (MAX_LA * 0.5);
                        linYawCruiseTime = totalYawTime - linYawAccelTime - linYawDeccelTime;
                        linYawCruiseDist = MAX_LV * 0.5 * linYawCruiseTime;
                    }
                    else
                    {
                        double ghostEndTime = (maxEndLinVel / (MAX_LA * 0.5));
                        linYawAccelTime = (ghostEndTime + totalYawTime) / 2;
                        linYawCruiseVel = linYawAccelTime * MAX_LA * 0.5;
                        linYawCruiseDist = 0;
                        linYawCruiseTime = 0;
                        linYawDeccelTime = linYawAccelTime - ghostEndTime;

                    }
                }
                else
                {
                    linYawAccelTime = (linMaxAccelTime > totalYawTime) ? totalYawTime : linMaxAccelTime;
                    linYawCruiseTime = (linMaxAccelTime > totalYawTime) ? 0 : totalYawTime - linMaxAccelTime;
                    linYawCruiseDist = linYawCruiseTime * MAX_LV * 0.5;
                    linYawCruiseVel = MAX_LV * 0.5;
                    linYawDeccelTime = 0;
                    endVel = linYawCruiseVel;
                }
            }

        }
        else if(dx == 0 && dy == 0 && dyaw != 0)
        {
            actualYawDist = 0;
            endVel = 0;
            yawCruiseVel = MAX_AA * sqrt(dyaw / MAX_AA);
            if(yawCruiseVel > MAX_AV)
            {
                yawCruiseVel = MAX_AV;
                yawAccelTime = MAX_AV / MAX_AA;
                double accelDeccelYaw = MAX_AV * yawAccelTime;
                yawCruiseDist = dyaw - accelDeccelYaw;
                yawCruiseTime = yawCruiseDist / MAX_AV;
            }
            else
            {
                yawAccelTime = yawCruiseVel / MAX_AA;
                yawCruiseDist = 0;
                yawCruiseTime = 0;
            }

            linYawAccelTime = 0;
            linYawCruiseTime = 0;
            linYawCruiseDist = 0;
            linYawCruiseVel = 0;
            linYawDeccelTime = 0;
        }
        else
        {
            actualYawDist = 0;
            endVel = 0;
            yawAccelTime = 0;
            yawCruiseTime = 0;
            yawCruiseDist = 0;
            yawCruiseVel = 0;
            linYawAccelTime = 0;
            linYawCruiseTime = 0;
            linYawCruiseDist = 0;
            linYawCruiseVel = 0;
            linYawDeccelTime = 0;
        }
        yawCruiseDist *= yawDirection;
        yawCruiseVel *= yawDirection;

        //Normal linear after yaw
        double newLinearDist = totalLinearDist - actualYawDist;
        double minDeccelDist = (endVel/2) * (endVel/MAX_LA);

        double linAccelTime, linCruiseTime, linDeccelTime, linCruiseVel, linCruiseDist;

        if(minDeccelDist >= newLinearDist)
        {
            cout << "end vel a little sus" << endl;
            linAccelTime = 0;
            linCruiseTime = 0;
            linDeccelTime = endVel / MAX_LA;
            linCruiseVel = 0;
            linCruiseDist = 0;
        }
        else
        {
            double ghostDist = minDeccelDist + newLinearDist;
            if(ghostDist / MAX_LA >= 0)
            {
                double maxCruiseVel = sqrt(ghostDist / MAX_LA) * MAX_LA;
                linCruiseVel = (maxCruiseVel > MAX_LV) ? MAX_LV : maxCruiseVel;

                linAccelTime = (linCruiseVel - endVel) / MAX_LA;
                linDeccelTime = linCruiseVel / MAX_LA;
                linCruiseDist = newLinearDist - (((endVel + linCruiseVel) / 2) * linAccelTime) - ((linCruiseVel / 2) * linDeccelTime);
                linCruiseTime = linCruiseDist / linCruiseVel;
            }
            else
            {
                linAccelTime = 0;
                linDeccelTime = 0;
                linCruiseDist = 0;
                linCruiseTime = 0;
                linCruiseVel = 0;
            }

        }

        SwerveTrajectory trajectory(p1, p2, yawAccelTime, yawCruiseTime, yawCruiseDist, yawCruiseVel, 
        linYawAccelTime, linYawCruiseTime, linYawCruiseDist, linYawCruiseVel, 
        linYawDeccelTime, actualYawDist, endVel, linAccelTime, linCruiseTime, linDeccelTime, linCruiseVel, linCruiseDist, yawDirection,
        MAX_LA, MAX_LV, MAX_AA, MAX_AV);

        trajectories_.push_back(trajectory);
        
    }
}

void SwervePath::generateSplineTrajectory()
{

}

void SwervePath::addPoint(SwervePose point)
{
    points_.push_back(point);
}

void SwervePath::reset()
{
    points_.clear();
    trajectories_.clear();
    /*klP_ = 0;
    klD_ = 0; 
    kaP_ = 0; 
    kaD_ = 0; 
    klV_ = 0; 
    klA_ = 0; 
    kaV_ = 0; 
    kaA_ = 0;*/
}

SwervePose* SwervePath::getPose(double time, bool& end)
{
    double trajectoryTime = 0;
    int trajectory = trajectories_.size() - 1;
    for (size_t i = 0; i < trajectories_.size(); ++i)
    {
        trajectoryTime += trajectories_[i].getTotalTime();
        end = (time > trajectoryTime && i == trajectories_.size() - 1);
        if (time < trajectoryTime || i == trajectories_.size() - 1)
        {
            trajectoryTime -= trajectories_[i].getTotalTime();
            trajectory = i;
            break;
        }
    }

    return new SwervePose(trajectories_[trajectory].getPose(time - trajectoryTime));
}