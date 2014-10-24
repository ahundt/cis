#ifndef _MATRIX_OPERATIONS_HPP_
#define _MATRIX_OPERATIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>


/// finds the average of all points in x,y,z
Eigen::VectorXd averagePoint(Eigen::MatrixXd a)
{
    Eigen::VectorXd ax = a.col(0);
    Eigen::VectorXd ay = a.col(1);
    Eigen::VectorXd az = a.col(2);
    Eigen::Vector3d abar;
    abar(0) = ax.mean(); abar(1) = ay.mean(); abar(2) = az.mean();
    return abar;
}

/// subtracts the average every point in x,y,z
/// this recenters the point cloud around 0
/// @todo instead of using a loop vectorize the calculation, can be done more efficiently
Eigen::MatrixXd centerPointsOnOrigin(Eigen::MatrixXd a)
{
    Eigen::VectorXd abar = averagePoint(a);
    for(int j=0; j<3;j++)
       for(int i=0; i<a.rows();i++)
           a(i,j)=a(i,j)-abar(j);
    return a;
}


#endif // _MATRIX_OPERATIONS_HPP_
