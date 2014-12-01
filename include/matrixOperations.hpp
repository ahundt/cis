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



/// Swap indexing order of vector of vectors, so if it is row major the returned vv will be column major.
/// @todo this is pretty inefficient, but the lengthts can vary. Maybe store the data differently
template<typename T>
const std::vector<std::vector<T> > swapIndexing(const std::vector<std::vector<T> >& uv){
    std::vector<std::vector<T> > vu;
    for(auto uvi : uv){
        int i = 0;
        for(auto vi :uvi){
            if(i == vu.size()) vu.push_back(std::vector<T>());
            vu[i].push_back(vi);
            ++i;
        }
    }
    return vu;
}

/// @brief combine a vector<vector<T> > into a single vector<T>
template<typename T>
const std::vector<T> concat(const std::vector<std::vector<T> >& uv){
    std::vector<T> u;
    for(auto uvi : uv){
        u.insert(u.end(), uvi.begin(), uvi.end());
    }
    return u;
}



/// @brief split a large Eigen::MatrixXd by rows into a vector of smaller MatrixXd
std::vector<Eigen::MatrixXd> splitRows(const Eigen::MatrixXd& mat,std::size_t numRowsPerMat){
    
    std::vector<Eigen::MatrixXd> vec;
    std::size_t cols = mat.cols();
    
    for(std::size_t currentRow = 0; currentRow < mat.rows(); currentRow+=numRowsPerMat){
        Eigen::MatrixXd partialMat(mat.block(currentRow,0,numRowsPerMat,cols));
        vec.push_back(partialMat);
    }
    
    return vec;
}

Eigen::MatrixXd concatToMatrix(const std::vector<Eigen::Vector3d>& points){
    Eigen::MatrixXd mat;
    mat.resize(points.size(),3);
    int i = 0;
    for(Eigen::Vector3d point : points){
        mat.block<1,3>(i,0) = point.transpose();
        ++i;
    }
    
    return mat;
}


// splits a stack of vectors that is nx3, into a std::vector of n 3x1 vectors
std::vector<Eigen::Vector3d> splitVectors(const Eigen::MatrixXd& mat){
	std::vector<Eigen::Vector3d> vec;
	
	for(int i = 0; i < mat.rows(); ++i){
		vec.push_back(Eigen::Vector3d(mat.block<1,3>(i,0).transpose()));
	}
	
	return vec;
}

#endif // _MATRIX_OPERATIONS_HPP_
