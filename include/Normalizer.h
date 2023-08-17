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


private:
    void normalizePoint();

    void normalizePose();

    ifstream pointFile;
    ifstream cameraFile;

    PointVec pointVec;
    PoseVec poseVec;

};


Eigen::Vector3d median(const Normalizer::PointVec &pointVec);

double median(std::vector<double> &pointAxiss);