#include "PoseVertex.h"
#include "Eigen/Core"

int main(int argc, const char **argv)
{
    Eigen::Vector3d angleAxis(-1.6943983532198115e-02, 3.1759419019880947e-14, 2.8396179128925302e-08);
    Eigen::Matrix3d R = angleAxis2R(angleAxis);
    std::cout << R * R.transpose() << std::endl;
    std::cout << R.determinant() << std::endl;
}

