#include "g2o/core/block_solver.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "VertexAndEdge.h"
#include "PosePointEdge.h"

#include <string>


const std::string POINT_FILE_PATH = "../res/PointVertexFile.txt";
const std::string POSE_FILE_PATH = "../res/CamVertexFile.txt";
const std::string EDGE_FILE_PATH = "../res/EdgeFile.txt";
const int MAX_ITERATIONS = 40;
const bool VERBOSE = true;

int main(int argc, const char **argv)
{
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>>;
    using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    g2o::SparseOptimizer graph;
    VertexAndEdge vertexAndEdge(POSE_FILE_PATH, POINT_FILE_PATH, EDGE_FILE_PATH);
    auto *algorithm = new g2o::OptimizationAlgorithmLevenberg(
            std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
    );
    graph.setAlgorithm(algorithm);
    graph.setVerbose(VERBOSE);

    vertexAndEdge.addVertexAndEdge(graph);

    graph.initializeOptimization();
    graph.optimize(MAX_ITERATIONS);

    return 0;
}