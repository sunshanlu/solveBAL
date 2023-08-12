#include <string>
#include <vector>
#include <fstream>
#include "g2o/core/sparse_optimizer.h"


class PointVertex;

class PoseVertex;

struct CamPoseType;

class VertexAndEdge
{
public:
    using string = std::string;
    using PointVec = std::vector<PointVertex *>;
    using PoseVec = std::vector<PoseVertex *>;
    using ifstream = std::ifstream;
    using SparseOptimizer = g2o::SparseOptimizer;

    VertexAndEdge(const string &poseFile, const string &pointFile, const string &edgeFile);

    void addVertexAndEdge(SparseOptimizer &graph)
    {
        addPointVertex(graph);
        addPoseVertex(graph);
        addEdge(graph);
    }

private:


    void addPoseVertex(SparseOptimizer &graph);

    void addPointVertex(SparseOptimizer &graph);

    void addEdge(SparseOptimizer &graph);

    PointVec pointVec;
    PoseVec poseVec;
    ifstream poseFile;
    ifstream pointFile;
    ifstream edgeFile;
};

std::istream &operator>>(std::istream &is, Eigen::Vector3d &point3d);

std::istream &operator>>(std::istream &is, CamPoseType &camPoseType);