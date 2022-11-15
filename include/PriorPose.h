#ifndef PRIORPOSETYPES_H
#define PRIORPOSETYPES_H

#include "Eigen/Core"
#include "sophus/se3.hpp"

namespace ORB_SLAM3
{
    struct PriorPose {
        PriorPose(const Sophus::SE3<float> pose, const double &timestamp):pose(pose), t(timestamp) {}
        double t;
        Sophus::SE3<float> pose;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //PRIORPOSETYPES_H