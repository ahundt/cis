#ifndef _DISTORTION_CALIBRATION_HPP_
#define _DISTORTION_CALIBRATION_HPP_

#include <boost/math/special_functions/binomial.hpp>
#include "matrixOperations.hpp"

//enum BoundingBoxRows {
//    max = 0,
//    min = 1
//};


/// Finds the maximum and minimum values of the point clouds in the X, Y,and Z
/// @todo what are the expected dimensions of X?
/// @todo create a unit test for this
///
/// Use this to find min and max:
///
/// Eigen::Vector3d MaxValues = X.colwise().maxCoeff();
/// Eigen::Vector3d MinValues = x.colwise().minCoeff();

//Eigen::MatrixXd FindMaxMinXYZ(const Eigen::MatrixXd& X)
//{
//    int cols = X.cols();
//    int rows = X.rows();
//    Eigen::MatrixXd maxmin(2,cols);
//    Eigen::VectorXd aVec;
//    for (int j = 0; j<cols; j++){
//        aVec = X.col(j);
//        double max = aVec(0);
//        double min = aVec(0);
//        for (int i = 0; i<rows; i++){
//            if (aVec(i) >= max) {
//                max=aVec(i);
//                maxmin(0,j)=max;
//            }
//            if (aVec(i) <= min) {
//                min=aVec(i);
//                maxmin(1,j)=min;
//            }
//        }
//    }
//    return maxmin;
//}

/// Scales every dimension of the point cloud to the max and min values
/// @todo what are the expected dimensions of X?
/// @todo which return row is the min and which is the max
/// @todo this does not scale to the unit box, it scales to the size of the max and min point of the matrix.
void ScaleToBox(Eigen::MatrixXd& X)
{
    Eigen::Vector3d max = X.colwise().maxCoeff();
    Eigen::Vector3d min = X.colwise().minCoeff();
    Eigen::Vector3d diff = max-min;
    for (int i=0; i<X.cols(); i++){
        for (int j=0; j<X.rows(); j++){
            // scale the x,y,z of the point
            X(j,i) = (X(j,i)-min(i))/diff(i);
        }
    }
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
/// @param N the polynomial degree
/// @see slide 42 and 43 of InterpolationReview.pdf
/// @todo advanced implementation: template on the polynomial size
Eigen::MatrixXd FMatrixRow(const Eigen::Vector3d& v,int N = 5, bool debug = false)
{
    int index = 0; // position in the output matrix
    int columns = pow(N+1,3);
    Eigen::MatrixXd F(1,columns);
    //std::cout << "\n\nF is " << F << std::endl;
    for (int i=0; i<=N; i++){
        for(int j=0; j<=N; j++){
            for(int k=0; k<=N; k++){
                BOOST_VERIFY(index<columns);
                // Fijk = Bi * Bj * Bk F(0,index)
                //std::cout << "\n\nindex is " << index << std::endl;
                double bSum = BersteinPolynomial(v(0),N,i)*BersteinPolynomial(v(1),N,j)*BersteinPolynomial(v(2),N,k);
                //std::cout << "\n\nB is " << B << std::endl;
                F.block<1,1>(0,index) << bSum;
                index++;
                //std::cout << "\n\nF is " << F << std::endl;
            }
        }
    }
    
    if(debug){
        std::cout << "\n\ncolumns is " << columns << std::endl;
        std::cout << "\n\nFMatrixRow:\n\n" << F << "\n\n";
    }
    return F;
}


/// @see slide 42 and 43 of InterpolationReview.pdf
/// @param cEM n x numPoints matrix containing the c expected value, aka actual points measured by EM tracker in EM coordinate system, after translation from EM coord system
/// @param N the polynomial degree
Eigen::MatrixXd FMatrix(const Eigen::MatrixXd& cEM, int N = 5, bool debug = false){
    /// @todo don't recompute pow here and in FMatrixRow
    int columns = pow(N+1,3);
    int rows = cEM.rows();
    Eigen::MatrixXd cEMFMatrix(rows,columns);
    
    for (int i=0; i<rows; i++){
        Eigen::Vector3d vXYZ;
        vXYZ = cEM.block<1,3>(i,0);
        Eigen::MatrixXd row = FMatrixRow(vXYZ,N,debug);
        if(debug) std::cout << "\n\nreturned FMatrixRow:\n\n" << row << "\n\n";
        cEMFMatrix.row(i) = row;
    }
    if(debug) std::cout << "\n\ncEMFMatrix:\n\n" << cEMFMatrix << "\n\n";
    return cEMFMatrix;
}

#endif // _DISTORTION_CALIBRATION_HPP_
