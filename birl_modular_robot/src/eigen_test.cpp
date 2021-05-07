#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

int main(int argc, char const *argv[])
{
    Eigen::Vector3d temp1(0,0,1);
    Eigen::Vector3d temp2;
    temp2 =  temp1;
    temp1(0) = 1;
    std::cout << temp1.matrix() << std::endl;
    std::cout << temp2.matrix() << std::endl;
    return 0;
}
