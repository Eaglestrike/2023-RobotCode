#pragma once

#include <math.h>

#include "Helpers/GeneralPoses.h"

namespace Poses{
    struct SwervePose{
        double x;
        double y;
        double yaw;
        double xVel = 0.0;
        double yVel = 0.0;
        double yawVel = 0.0;
        double xAcc = 0.0;
        double yAcc = 0.0;
        double yawAcc = 0.0;
    };

    /// @brief Linear distance from start to finish
    /// @param startPose start pose
    /// @param endPose end pose
    /// @return distance
    static double SwervePoseDist(SwervePose startPose, SwervePose endPose)
    {
        double dx = endPose.x - startPose.x;
        double dy = endPose.y - startPose.y;
        return sqrt(dx*dx + dy*dy);
    }

    /// @brief gets the angle from start pose to finish pose, locationwise
    /// @param startPose starting pose
    /// @param endPose ending pose
    /// @return ang from start to finish in degrees
    static double SwervePoseAng(SwervePose startPose, SwervePose endPose)
    {
        double dx = endPose.x - startPose.x;
        double dy = endPose.y - startPose.y;
        if(dx != 0 || dy != 0)
        {
            return 90 - (atan2(dy, dx) * 180 / M_PI);
        }
        else
        {
            return 0;
        }
    }

    /// @brief Combines 1D poses into 3D swerve pose
    /// @param xPose Pose1D of x component
    /// @param yPose Pose1D of y component
    /// @param angPose Pose1D of angular component
    /// @return Combined SwervePose
    static inline SwervePose SwerveFromPose1D(Pose1D xPose, Pose1D yPose, Pose1D angPose){
        return SwervePose{
            xPose.pos,
            yPose.pos,
            angPose.pos,
            xPose.vel,
            yPose.vel,
            angPose.vel,
            xPose.acc,
            yPose.acc,
            angPose.acc
        };
    }

    /// @brief returns if any of the velocities aren't zero
    /// @param pose SwervePose
    /// @return if the pose is moving
    static inline bool isMoving(SwervePose pose){
        return (pose.xVel != 0 || pose.yVel != 0 || pose.yawVel != 0);
    }

    /// @brief Returns if all dimensions are stopped
    /// @param pose SwervePose
    /// @return if the pose is stationary
    static inline bool isStationary(SwervePose pose){
        return pose.xVel == 0 && pose.yVel == 0 && pose.yawVel == 0 &&
               pose.xAcc == 0 && pose.yAcc == 0 && pose.yawAcc == 0;
    }
};