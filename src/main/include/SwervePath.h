#pragma once

#include "vector"
#include "SwerveTrajectory.h"
#include "SwervePose.h"
#include "Helpers.h"
#include "math.h"
#include "iostream"

using namespace std;

class SwervePath
{
    public:
        SwervePath(double maxLA, double maxLV, double maxAA, double maxAV);
        void setKLP(double klP);
        void setKLD(double klD);
        void setKAP(double kaP);
        void setKAD(double kaD);

        void setKLV(double klV);
        void setKLA(double klA);
        void setKAV(double kaV);
        void setKAA(double kaA);

        void generateTrajectory(bool spline);
        void generateLinearTrajectory();
        void generateSplineTrajectory();
        void addPoint(SwervePose point);
        
        void reset();

        SwervePose* getPose(double time, bool& end);

    private:
        double MAX_LA, MAX_LV, MAX_AA, MAX_AV, klP_, klD_, kaP_, kaD_, klV_, klA_, kaV_, kaA_;

        vector<SwervePose> points_;
        vector<SwerveTrajectory> trajectories_;
};