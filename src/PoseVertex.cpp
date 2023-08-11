#include "PoseVertex.h"

bool PoseVertex::read(std::istream &is)
{
    return false;
}

bool PoseVertex::write(std::ostream &os) const
{
    return false;
}

void PoseVertex::setToOriginImpl()
{
    _estimate.pose = Sophus::SE3d();
    _estimate.camParams = Eigen::Vector3d::Zero();
}

void PoseVertex::oplusImpl(const number_t *v)
{
    Eigen::Vector3d angleAxis = Eigen::Vector3d::Map(v);
    Eigen::Vector3d t = Eigen::Vector3d::Map(v + 3);
    Eigen::Vector3d camParamUpdate = Eigen::Vector3d::Map(v + 6);
    Eigen::Matrix3d R = angleAxis2R(angleAxis);

    Sophus::SE3d updatePose(R, t);
    _estimate.pose = updatePose * _estimate.pose;
    _estimate.camParams += camParamUpdate;
}


Eigen::Matrix3d hat(const Eigen::Vector3d &vectorR)
{
    Eigen::Matrix3d vectorRHat;
    vectorRHat << 0, -vectorR(2), vectorR(1),
            vectorR(2), 0, -vectorR(0),
            -vectorR(1), vectorR(0), 0;
    return vectorRHat;
}


Eigen::Matrix3d angleAxis2R(const Eigen::Vector3d &angleAxis)
{
    double theta = angleAxis.norm();
    Eigen::Matrix3d R =
            std::cos(theta) * Eigen::Matrix3d::Identity() +
            (1 - std::cos(theta)) * angleAxis * angleAxis.transpose() + sin(theta) * hat(angleAxis);

    return R;
}