#pragma once

#include "g2o/core/base_vertex.h"
#include "sophus/se3.hpp"

struct CamPoseType
{
    Sophus::SE3d pose;
    Eigen::Vector3d camParams;
};

class PoseVertex : public g2o::BaseVertex<9, CamPoseType>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PoseVertex() = default;

    bool read(std::istream &is) override;

    bool write(std::ostream &os) const override;

    void setToOriginImpl() override;

    void oplusImpl(const number_t *v) override;

};


Eigen::Matrix3d angleAxis2R(const Eigen::Vector3d &angleAxis);