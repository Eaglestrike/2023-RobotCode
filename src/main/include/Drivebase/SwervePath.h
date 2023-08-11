#pragma once

#include "vector"
#include "math.h"
#include "iostream"

#include "SwerveTrajectory.h"
#include "SwervePose.h"

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
        void addPoint(Poses::SwervePose point);
        
        void reset();

        Poses::SwervePose getPose(double time, bool& end);

    private:
        double MAX_LA, MAX_LV, MAX_AA, MAX_AV, klP_, klD_, kaP_, kaD_, klV_, klA_, kaV_, kaA_;

        std::vector<Poses::SwervePose> points_;
        std::vector<SwerveTrajectory> trajectories_;
};