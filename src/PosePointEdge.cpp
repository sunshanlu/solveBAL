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
    const auto *pointVertex = dynamic_cast<const PointVertex *>(_vertices[1]);
    const auto *poseVertex = dynamic_cast<const PoseVertex *>(_vertices[0]);

    const auto camParams = poseVertex->estimate().camParams;
    const auto f = camParams.x();
    const auto k1 = camParams.y();
    const auto k2 = camParams.z();

    Eigen::Vector3d cameraPoint = poseVertex->estimate().pose * pointVertex->estimate();
    Eigen::Vector2d point2d = reProjection(cameraPoint, f, k1, k2);

    _error = point2d - error();

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
    double distortCoef = (1 + k1 * std::pow(r, 2) + k2 * std::pow(r, 4));
    point2d *= distortCoef;
    // step 3: Pixel coordinate system
    point2d *= f;
    return point2d;
}