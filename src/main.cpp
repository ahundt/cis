#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include "matrixOperations.hpp"
#include "hornRegistration.hpp"


int main()
{
   Eigen::Matrix3d a;
    a << 1, 2, 3,
     2, 4, 6,
     3, 6, 9;

     Eigen::Matrix3d b;
    b << 1, 2, 3,
     4, 5, 6,
     5, 8, 9;

    Eigen::Matrix4d F = hornRegistration(a,b);
    std::cout << F << std::endl;
}
