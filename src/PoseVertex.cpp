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
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    Vector6d updatePose;
    updatePose << v[0], v[1], v[2], v[3], v[4], v[5];
    Eigen::Vector3d camParamUpdate;
    camParamUpdate << v[6], v[7], v[8];

    _estimate.pose = Sophus::SE3d::exp(updatePose) * _estimate.pose;
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


Eigen::Quaterniond angleAxis2Q(const Eigen::Vector3d &angleAxis)
{
    double theta = angleAxis.norm();
    double sinHalfTheta = std::sin(theta / 2);

    double data[4];

    data[3] = std::cos(theta / 2);
    data[0] = angleAxis(0) * sinHalfTheta;
    data[1] = angleAxis(1) * sinHalfTheta;
    data[2] = angleAxis(2) * sinHalfTheta;
    return Eigen::Quaterniond(data);
}