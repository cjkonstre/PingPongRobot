#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::Vector3d v(1,2,3);
    Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
    std::cout << "M*v = " << (M*v).transpose() << "\n";
}
