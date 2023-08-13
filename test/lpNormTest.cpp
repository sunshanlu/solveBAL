#include "Eigen/Core"
#include <iostream>

int main()
{
    Eigen::Vector3d vector3D(1, 2, -3);
    std::cout << vector3D.lpNorm<1>() << std::endl;
}