#include "VertexAndEdge.h"
#include "Eigen/Core"
#include "PoseVertex.h"
#include "PosePointEdge.h"
#include "g2o/core/robust_kernel_impl.h"

std::istream &operator>>(std::istream &is, Eigen::Vector3d &point3d)
{
    double px, py, pz;
    if (!(is >> px))
        return is;
    is >> py >> pz;
    point3d << px, py, pz;
    return is;
}

std::istream &operator>>(std::istream &is, CamPoseType &camPoseType)
{
    double rx, ry, rz, tx, ty, tz, f, k1, k2;
    if (!(is >> rx))
        return is;
    is >> ry >> rz >> tx >> ty >> tz >> f >> k1 >> k2;
    Eigen::Vector3d angleAxis(rx, ry, rz);
    Eigen::Vector3d t(tx, ty, tz);
    Eigen::Vector3d camParams(f, k1, k2);
    auto R = angleAxis2Q(angleAxis);
    camPoseType.pose = Sophus::SE3d(R, t);
    camPoseType.camParams = camParams;

    return is;
}


VertexAndEdge::VertexAndEdge(const Normalizer *normalizer, const string &edgeFile)
        : normalizer(normalizer), edgeFile(edgeFile)
{

}

void VertexAndEdge::addEdge(SparseOptimizer &graph)
{
    int camID, pointID, edgeID = 0;
    double px, py;
    Eigen::Vector2d pixelPoint;
    while (edgeFile >> camID) {
        edgeFile >> pointID >> px >> py;
        pixelPoint << px, py;
        auto *edge = new PosePointEdge;
        edge->setId(edgeID);
        edge->setVertex(0, poseVec[camID]);
        edge->setVertex(1, pointVec[pointID]);
        edge->setMeasurement(pixelPoint);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        graph.addEdge(edge);

        ++edgeID;
    }
}



