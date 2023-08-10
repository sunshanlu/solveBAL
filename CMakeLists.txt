cmake_minimum_required(VERSION 3.26)
project(solveBAL)

set(CMAKE_CXX_STANDARD 14)
SET(G2O_LIBS g2o_stuff g2o_core cxsparse)

find_package(Sophus REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)

include_directories(
        include
        ${Sophus_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        /usr/include/eigen3
        ${GLOG_INCLUDE_DIRS}
)

add_executable(
        ${PROJECT_NAME}
        src/main.cpp
        src/PointVertex.cpp
        src/PoseVertex.cpp
        src/PosePointEdge.cpp
        include/PosePointEdge.h
)
target_link_libraries(
        ${PROJECT_NAME}
        Sophus::Sophus
        ${G2O_LIBS}
        ${GLOG_LIBRARIES}
)