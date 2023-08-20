#include "g2o/core/block_solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "VertexAndEdge.h"
#include "PosePointEdge.h"
#include "Normalizer.h"

#include <string>

const std::string POINT_FILE_PATH = "../res/PointVertexFile.txt";
const std::string POSE_FILE_PATH = "../res/CamVertexFile.txt";
const std::string EDGE_FILE_PATH = "../res/EdgeFile.txt";
const std::string INIT_PLY_FILE_PATH = "../result/Init.ply";
const std::string FINAL_PLY_FILE_PATH = "../result/Final.ply";
const int MAX_ITERATIONS = 40;
const bool VERBOSE = true;

int main(int argc, const char **argv)
{
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    g2o::SparseOptimizer graph;

    Normalizer normalizer(POINT_FILE_PATH, POSE_FILE_PATH);
    normalizer.normalize();
    writePLYFile(INIT_PLY_FILE_PATH, normalizer);

    VertexAndEdge vertexAndEdge(&normalizer, EDGE_FILE_PATH);
    auto *algorithm = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
    graph.setAlgorithm(algorithm);
    graph.setVerbose(VERBOSE);

    addVertexAndEdge(vertexAndEdge, graph);

    graph.initializeOptimization();
    graph.optimize(MAX_ITERATIONS);

    vertexAndEdge.toNormalizer();
    writePLYFile(FINAL_PLY_FILE_PATH, normalizer, false);

    return 0;
}