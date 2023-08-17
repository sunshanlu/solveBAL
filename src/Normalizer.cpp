#include "Normalizer.h"
#include "VertexAndEdge.h"

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
        poseVec.push_back(camPose);
    }
    normalizePose();
}

void Normalizer::normalizePose()
{

}

void Normalizer::normalizePoint()
{
    Eigen::Vector3d medianPoint = median(pointVec);

    std::vector<double> l1NormValues;
    for (const auto &point: pointVec) {
        l1NormValues.emplace_back((point - medianPoint).lpNorm<1>());
    }
    const double scale = 100.0 / median(l1NormValues);
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

double median(std::vector<double> &pointAxiss)
{
    int midNum = static_cast<int>(pointAxiss.size()) / 2;
    auto midIter = pointAxiss.begin() + midNum;
    std::nth_element(pointAxiss.begin(), midIter, pointAxiss.end());
    return *midIter;
}