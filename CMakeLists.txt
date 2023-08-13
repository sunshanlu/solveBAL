cmake_minimum_required(VERSION 3.5)
project(solveBAL)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(G2O_LIBS g2o_stuff g2o_core)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(G2O REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(
        ${PROJECT_NAME}
        src/main.cpp
        src/PointVertex.cpp
        src/PoseVertex.cpp
        src/PosePointEdge.cpp
        src/VertexAndEdge.cpp
)
target_link_libraries(
        ${PROJECT_NAME}
        Sophus::Sophus
        ${G2O_LIBS}
)

add_executable(
        angleAxis2RTest
        test/angleAxis2RTest.cpp
        src/PoseVertex.cpp
)

target_link_libraries(
        angleAxis2RTest
        Sophus::Sophus
        ${G2O_LIBS}
)
