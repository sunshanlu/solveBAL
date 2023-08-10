#include "PointVertex.h"


bool PointVertex::read(std::istream &is)
{
    return false;
}

bool PointVertex::write(std::ostream &os) const
{
    return false;
}

void PointVertex::oplusImpl(const number_t *v)
{
    Eigen::Vector3d update = Eigen::Vector3d::Map(v);
    _estimate += update;
}

void PointVertex::setToOriginImpl()
{
    _estimate = Eigen::Vector3d::Zero();
}
