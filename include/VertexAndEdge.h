#pragma once

#include <string>
#include <vector>
#include <fstream>
#include "g2o/core/sparse_optimizer.h"


class PointVertex;

class PoseVertex;

struct CamPoseType;

class Normalizer;

class VertexAndEdge
{
    friend void addVertexAndEdge(VertexAndEdge &, g2o::SparseOptimizer &);

public:
    using string = std::string;
    using PointVec = std::vector<PointVertex *>;
    using PoseVec = std::vector<PoseVertex *>;
    using ifstream = std::ifstream;
    using SparseOptimizer = g2o::SparseOptimizer;

    VertexAndEdge(Normalizer *normalizer, const string &edgeFile);

    void toNormalizer();

private:

    void addPoseVertex(SparseOptimizer &graph);

    void addPointVertex(SparseOptimizer &graph);

    void addEdge(SparseOptimizer &graph);

    PointVec pointVec;
    PoseVec poseVec;
    Normalizer *normalizer;
    ifstream edgeFile;
};

std::istream &operator>>(std::istream &is, Eigen::Vector3d &point3d);

std::istream &operator>>(std::istream &is, CamPoseType &camPoseType);

void addVertexAndEdge(VertexAndEdge &vertexAndEdge, g2o::SparseOptimizer &graph);