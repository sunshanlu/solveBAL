#pragma once

#include "g2o/core/base_vertex.h"
#include "Eigen/Core"

class PointVertex : public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointVertex() = default;

    bool read(std::istream &is) override;

    bool write(std::ostream &os) const override;

    void oplusImpl(const number_t *v) override;

    void setToOriginImpl() override;

};

