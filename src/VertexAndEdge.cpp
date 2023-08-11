#include "VertexAndEdge.h"
#include "Eigen/Core"
#include "PointVertex.h"
#include "PoseVertex.h"
#include "PosePointEdge.h"

std::istream &operator>>(std::istream &is, Eigen::Vector3d &point3d)
{
    double px, py, pz;
    is >> px >> py >> pz;
    point3d << px, py, pz;
    return is;
}

std::istream &operator>>(std::istream &is, CamPoseType &camPoseType)
{
    double rx, ry, rz, tx, ty, tz, f, k1, k2;
    is >> rx >> ry >> rz >> tx >> ty >> tz >> f >> k1 >> k2;
    Eigen::Vector3d angleAxis(rx, ry, rz);
    Eigen::Vector3d t(tx, ty, tz);
    Eigen::Vector3d camParams(f, k1, k2);
    auto R = angleAxis2R(angleAxis);
    camPoseType.pose = Sophus::SE3d(R, t);
    camPoseType.camParams = camParams;

    return is;
}


VertexAndEdge::VertexAndEdge(const string &poseFile, const string &pointFile, const string &edgeFile)
        : poseFile(poseFile), pointFile(pointFile), edgeFile(edgeFile)
{

}

void VertexAndEdge::addPointVertex(SparseOptimizer &graph)
{
    Eigen::Vector3d worldPoint;
    int pointID = 0;
    while (pointFile) {
        pointFile >> worldPoint;
        auto *pointVertex = new PointVertex;
        pointVertex->setId(pointID);
        pointVertex->setEstimate(worldPoint);
        graph.addVertex(pointVertex);
        pointVec.push_back(pointVertex);

        ++pointID;
    }
}

void VertexAndEdge::addPoseVertex(SparseOptimizer &graph)
{
    CamPoseType camPoseType;
    int poseID = 0;
    while (poseFile) {
        poseFile >> camPoseType;
        auto *poseVertex = new PoseVertex;
        poseVertex->setId(poseID);
        poseVertex->setEstimate(camPoseType);
        graph.addVertex(poseVertex);
        poseVec.push_back(poseVertex);

        ++poseID;
    }
}

void VertexAndEdge::addEdge(SparseOptimizer &graph)
{
    int camID, pointID, edgeID = 0;
    double px, py;
    Eigen::Vector2d pixelPoint;
    while (edgeFile) {
        edgeFile >> camID >> pointID >> px >> py;
        pixelPoint << px, py;
        auto *edge = new PosePointEdge;
        edge->setId(edgeID);
        edge->setVertex(0, poseVec[camID]);
        edge->setVertex(1, pointVec[pointID]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setMeasurement(pixelPoint);
        graph.addEdge(edge);

        ++edgeID;
    }
}



