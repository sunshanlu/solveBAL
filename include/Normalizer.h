#pragma once

#include <fstream>
#include <vector>

#include "Eigen/Core"
#include "PoseVertex.h"

class Normalizer
{
public:
    using ifstream = std::ifstream;
    using PointVec = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
    using PoseVec = std::vector<CamPoseType>;

    Normalizer(const std::string &pointFile, const std::string &cameraFile);

    Normalizer() = delete;

    void normalize();

    const PointVec &getPointVec() const
    {
        return pointVec;
    }

    const PoseVec &getPoseVec() const
    {
        return poseVec;
    }


private:
    void normalizePoint();


    ifstream pointFile;
    ifstream cameraFile;

    PointVec pointVec;
    PoseVec poseVec;

    double scale{};

    Eigen::Vector3d medianPoint;

};


Eigen::Vector3d median(const Normalizer::PointVec &pointVec);

double median(std::vector<double> &pointAxiss);

Eigen::Vector3d trans2Center(const Eigen::Vector3d &trans, const Sophus::SO3d &so3D);

Eigen::Vector3d center2Trans(const Eigen::Vector3d &centerNorm, const Sophus::SO3d &so3D);