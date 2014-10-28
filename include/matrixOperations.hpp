#ifndef _MATRIX_OPERATIONS_HPP_
#define _MATRIX_OPERATIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace CIS {
/// @brief Manual transform implementation, recommend using the following instead
/// 
/// @verbatim
///    Eigen::Affine3d transform;
///    
///    transform.setIdentity();
///    
///    // Define a translation of 2.5 meters on the x axis.
///    transform.translation() << 2.5, 0.0, 0.0;
///    
///    double theta = 0;//boost::math::constants::pi<double>();
///    // The same rotation matrix as before; tetha radians arround Z axis
///    transform.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
///    
///    Eigen::Vector3d v3d(1,1,1);
///
///    Eigen::Vector3d v3d2 = (transform*v3d).transpose();
/// @endverbatim
template<typename Scalar>
Eigen::Matrix<Scalar, 3, 1> transform(const Eigen::Transform<Scalar, 3, Eigen::Affine>& hTrans, const Eigen::Matrix<Scalar, 3, 1>& point){
   return Eigen::Matrix<Scalar, 3, 1>
    (
        hTrans (0, 0) * point(0) + hTrans (0, 1) * point(1) + hTrans (0, 2) * point(2) + hTrans (0, 3),
        hTrans (1, 0) * point(0) + hTrans (1, 1) * point(1) + hTrans (1, 2) * point(2) + hTrans (1, 3),
        hTrans (2, 0) * point(0) + hTrans (2, 1) * point(1) + hTrans (2, 2) * point(2) + hTrans (2, 3)
     );

}
}

/// @todo provide instructions to replace this with eigen built in functionality
/// Create transformation matrix from Rotation R and Translation vector P
Eigen::Matrix4d homogeneousmatrix(Eigen::Matrix3d R, Eigen::Vector3d p)
{
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F.block<3,3>(0,0) = R;
    F.block<3,1>(0,3) = p;
    return F;
}

/// @todo provide instructions to replace this with eigen built in functionality
/// Computes the inverse of the transformation matrix
Eigen::MatrixXd homogeneousInverse(const Eigen::MatrixXd& F)
{
    Eigen::MatrixXd Finv = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = F.block<3,3>(0,0);
    Eigen::Matrix3d Rtrans = R.transpose();
    Finv.block<3,3>(0,0) = Rtrans;
    Finv.block<3,1>(0,3) = -Rtrans*F.block<3,1>(0,3);
    return Finv;
}


#endif // _MATRIX_OPERATIONS_HPP_
