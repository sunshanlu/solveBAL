#pragma once

#include "g2o/core/base_binary_edge.h"
#include "PointVertex.h"
#include "PoseVertex.h"

class PosePointEdge : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, PoseVertex, PointVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PosePointEdge() = default;

    bool read(std::istream &is) override;

    bool write(std::ostream &os) const override;

    void computeError() override;
};

Eigen::Vector2d reProjection(const Eigen::Vector3d &cameraPoint, const double &f, const double &k1, const double &k2);
