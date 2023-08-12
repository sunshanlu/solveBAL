#include "PosePointEdge.h"

bool PosePointEdge::read(std::istream &is)
{
    return false;
}

bool PosePointEdge::write(std::ostream &os) const
{
    return false;
}

void PosePointEdge::computeError()
{
    auto *pointVertex = dynamic_cast<PointVertex *>(_vertices[1]);
    auto *poseVertex = dynamic_cast<PoseVertex *>(_vertices[0]);

    auto &camParams = poseVertex->estimate().camParams;
    auto &f = camParams.x();
    auto &k1 = camParams.y();
    auto &k2 = camParams.z();

    Eigen::Vector3d cameraPoint = poseVertex->estimate().pose * pointVertex->estimate();
    Eigen::Vector2d point2d = reProjection(cameraPoint, f, k1, k2);

    _error = point2d - _measurement;
}


Eigen::Vector2d reProjection(const Eigen::Vector3d &cameraPoint, const double &f, const double &k1, const double &k2)
{
    // step 1: Normalized Coordinate System
    Eigen::Vector3d point3d = -cameraPoint / cameraPoint(2);
    // step 2: Remove image distortion
    Eigen::Vector2d point2d(
            point3d.x(), point3d.y()
    );
    double r = point2d.norm();
    double distortCoef = (1.0 + k1 * std::pow(r, 2.0) + k2 * std::pow(r, 4.0));
    point2d *= distortCoef;
    // step 3: Pixel coordinate system
    point2d *= f;
    return point2d;
}