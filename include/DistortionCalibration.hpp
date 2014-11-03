#ifndef _DISTORTION_CALIBRATION_HPP_
#define _DISTORTION_CALIBRATION_HPP_

#include <boost/math/special_functions/binomial.hpp>
#include "matrixOperations.hpp"

/// Finds the maximum and minimum values of the point clouds in the X, Y,and Z
/// @todo what are the expected dimensions of X?
Eigen::MatrixXd FindMaxMinXYZ(Eigen::MatrixXd X)
{
    Eigen::MatrixXd maxmin(2,3);
    Eigen::VectorXd aVec;
    for (int j = 0; j<3; j++){
        aVec = X.col(j);
        double max = aVec(0);
        double min = aVec(0);
        for (int i = 0; i<aVec.rows(); i++){
            if (aVec(i) >= max) {
                max=aVec(i);
                maxmin(0,j)=max;
            }
            if (aVec(i) <= min) {
                min=aVec(i);
                maxmin(1,j)=min;
            }
        }
    }
    return maxmin;
}

/// Scales every dimension of the point cloud to the max and min values
/// @todo what are the expected dimensions of X?
Eigen::MatrixXd ScaleToBox(Eigen::MatrixXd X)
{
    Eigen::MatrixXd MaxMinValues = FindMaxMinXYZ(X);
    for (int i=0; i<3; i++){
        for (int j=0; j<X.rows(); j++){
            X(j,i) = (X(j,i)-MaxMinValues(1,i))/(MaxMinValues(0,i)-MaxMinValues(1,i));
        }
    }
    return X;
}

template <class T>
T boost::math::binomial_coefficient(unsigned n, unsigned k);

double BersteinPolynomial(double v, int N, int k)
{
    BOOST_VERIFY(N>=k);
    double B = boost::math::binomial_coefficient<double>(N,k)*pow(1-v,N-k)*pow(v,k);
    return B;
}

/// Makes F matrix of Berstein Polynomials
/// For
Eigen::MatrixXd FMatrix(Eigen::Vector3d v)
{
    int N = 5;
    int index = 0;
    int columns = pow(N+1,3)-1;
    Eigen::MatrixXd F(1,columns);
    //std::cout << "\n\nF is " << F << std::endl;
    for (int i=0; i<=5; i++){
        for(int j=0; j<=5; j++){
            for(int k=0; k<=5; k++){
                // Fijk = Bi * Bj * Bk F(0,index)
                //std::cout << "\n\nindex is " << index << std::endl;
                double bSum = BersteinPolynomial(v(0),N,i)*BersteinPolynomial(v(1),N,j)*BersteinPolynomial(v(2),N,k);
                //std::cout << "\n\nB is " << B << std::endl;
                F.block<1,1>(0,index) << bSum;
                index++;
                BOOST_VERIFY(index<columns);
                //std::cout << "\n\nF is " << F << std::endl;
            }
        }
    }
    std::cout << "\n\ncolumns is " << columns << std::endl;
    return F;
}

#endif // _DISTORTION_CALIBRATION_HPP_
