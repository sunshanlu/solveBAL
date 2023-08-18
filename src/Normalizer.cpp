#include "Normalizer.h"
#include "VertexAndEdge.h"
#include "PointVertex.h"

#include <algorithm>

Normalizer::Normalizer(const std::string &pointFile, const std::string &cameraFile)
        : pointFile(pointFile),
          cameraFile(cameraFile)
{

}

void Normalizer::normalize()
{
    Eigen::Vector3d worldPoint;
    while (pointFile >> worldPoint) {
        pointVec.push_back(worldPoint);
    }
    normalizePoint();

    CamPoseType camPose;
    while (cameraFile >> camPose) {
        Eigen::Vector3d &trans = camPose.pose.translation();
        Eigen::Vector3d center = trans2Center(trans, camPose.pose.so3());
        Eigen::Vector3d centerNorm = scale * (center - medianPoint);

        centerVec.push_back(centerNorm);
        trans = center2Trans(centerNorm, camPose.pose.so3());
        poseVec.push_back(camPose);
    }
}

void Normalizer::normalizePoint()
{
    medianPoint = median(pointVec);

    std::vector<double> l1NormValues;
    for (const auto &point: pointVec) {
        l1NormValues.emplace_back((point - medianPoint).lpNorm<1>());
    }
    scale = 100.0 / median(l1NormValues);
    for (auto &point: pointVec) {
        point = scale * (point - medianPoint);
    }
}

Eigen::Vector3d median(const Normalizer::PointVec &pointVec)
{
    std::vector<double> pointXs;
    std::vector<double> pointYs;
    std::vector<double> pointZs;

    for (const auto &point: pointVec) {
        pointXs.emplace_back(point.x());
        pointYs.emplace_back(point.y());
        pointZs.emplace_back(point.z());
    }
    return {median(pointXs),
            median(pointYs),
            median(pointZs)
    };
}

Eigen::Vector3d trans2Center(const Eigen::Vector3d &trans, const Sophus::SO3d &so3D)
{
    Eigen::Matrix3d R = so3D.matrix();
    const Eigen::Matrix3d RInverse = R.inverse();
    return -RInverse * trans;
}


Eigen::Vector3d center2Trans(const Eigen::Vector3d &centerNorm, const Sophus::SO3d &so3D)
{
    Eigen::Matrix3d R = so3D.matrix();
    return -R * centerNorm;
}


double median(std::vector<double> &pointAxiss)
{
    int midNum = static_cast<int>(pointAxiss.size()) / 2;
    auto midIter = pointAxiss.begin() + midNum;
    std::nth_element(pointAxiss.begin(), midIter, pointAxiss.end());
    return *midIter;
}

void VertexAndEdge::addPointVertex(SparseOptimizer &graph)
{
    int pointID = 16;
    for (const auto &worldPoint: normalizer->getPointVec()) {
        auto *pointVertex = new PointVertex;
        pointVertex->setId(pointID);
        pointVertex->setEstimate(worldPoint);
        pointVertex->setMarginalized(true);
        graph.addVertex(pointVertex);
        pointVec.push_back(pointVertex);

        ++pointID;
    }
}

void VertexAndEdge::addPoseVertex(SparseOptimizer &graph)
{
    int poseID = 0;
    for (const auto &camPoseType: normalizer->getPoseVec()) {
        auto *poseVertex = new PoseVertex;
        poseVertex->setId(poseID);
        poseVertex->setEstimate(camPoseType);
        graph.addVertex(poseVertex);
        poseVec.push_back(poseVertex);

        ++poseID;
    }
}

void writePLYFile(const std::string &plyFile, Normalizer &normalizer, const bool &init)
{
    if (not init) {
        std::size_t id = 0;
        for (auto &pose: normalizer.getPoseVec()) {
            const Eigen::Vector3d trans = pose.pose.translation();
            const Sophus::SO3d so3D = pose.pose.so3();
            normalizer.getCenterVec()[id] = trans2Center(trans, so3D);

            ++id;
        }
    }

    std::ofstream os(plyFile);
    const Normalizer::CenterVec &centerVec = normalizer.getCenterVec();
    const Normalizer::PointVec &pointVec = normalizer.getPointVec();
    std::size_t pointsNum = centerVec.size() + pointVec.size();
    os << "ply" << '\n'
       << "format ascii 1.0" << '\n'
       << "element vertex " << pointsNum << '\n'
       << "property float x" << '\n'
       << "property float y" << '\n'
       << "property float z" << '\n'
       << "property uchar red" << '\n'
       << "property uchar green" << '\n'
       << "property uchar blue" << '\n'
       << "end_header" << std::endl;

    for (const auto &center: centerVec)
        os << center.x() << " " << center.y() << " " << center.z() << " " << "0 255 0\n";

    for (const auto &point: pointVec)
        os << point.x() << " " << point.y() << " " << point.z() << " " << "255 255 255\n";
}

void VertexAndEdge::toNormalizer()
{
    Normalizer::PointVec &points = normalizer->getPointVec();
    Normalizer::PoseVec &poses = normalizer->getPoseVec();
    for (std::size_t id = 0; id < pointVec.size(); id++) {
        points[id] = pointVec[id]->estimate();
    }
    for (std::size_t id = 0; id < poseVec.size(); ++id) {
        poses[id] = poseVec[id]->estimate();
    }
}