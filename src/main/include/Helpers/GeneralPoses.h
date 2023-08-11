#pragma once

namespace Poses{
    struct Pose1D{
        double pos;
        double vel;
        double acc;
    };

    /// @brief Returns if everything is zero
    /// @param pose given pose
    /// @return if everything is zero
    inline bool isZero(Pose1D pose){
        return (pose.acc == 0.0) &&
               (pose.vel == 0.0) &&
               (pose.pos == 0.0);
    }

    /// @brief Returns if not acceleration or moving
    /// @param pose given pose
    /// @return bool if pose has stopped
    inline bool isStationary(Pose1D pose){
        return (pose.acc == 0.0) &&
               (pose.vel == 0.0);
    }
}