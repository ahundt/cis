#ifndef _HORN_REGISTRATION_HPP_
#define _HORN_REGISTRATION_HPP_

#include "matrixOperations.hpp"

/// creates Hmatrix for Horn's method
Eigen::Matrix3d Hmatrix(Eigen::MatrixXd a, Eigen::MatrixXd b)
{
    Eigen::Matrix3d H;
    Eigen::Matrix3d Hsum = Eigen::Matrix3d::Zero(3,3);
    for (int k=0; k<a.cols(); k++){
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++){
                H(i,j)=a(k,i)*b(k,j);
            }
        Hsum = Hsum+H;
    }
    return Hsum;
}

/// creates Gmatrix from Hmatrix using Horn's method
Eigen::Matrix4d Gmatrix(Eigen::Matrix3d H)
{
    Eigen::Vector3d delta;
    delta(0) = H(1,2)-H(2,1);
    delta(1) = H(2,0)-H(0,2);
    delta(2) = H(0,1)-H(1,0);
    Eigen::Matrix4d G;
    G(0,0)= H.trace();
    G.block<3,1>(1,0) = delta;
    G.block<1,3>(0,1) = delta.transpose();
    G.block<3,3>(1,1) = H+H.transpose()-H.trace()*Eigen::Matrix3d::Identity();
    return G;
}

/// Standard Library Comparator implementation,
/// used to sort the eigenvalues and corresponding eigenvector
struct SortPairsFirstHighestToLowest {
template <typename K, typename V>
bool operator()(const std::pair<K,V>& lhs, const std::pair<K,V>& rhs)
{
  return lhs.first > rhs.first;
}
};

/// Need to find the largest eigenvalue and the corresponding eigenvector
/// of the Gmatrix.
/// The eigenvector corresponding to the largest eigenvalue is the quaternion of the rotation matrix
/// @return quaternion representing the rotation
Eigen::Quaternion<double> EigenMatrix(Eigen::Matrix4d G)
{
    Eigen::EigenSolver<Eigen::Matrix4d> es(G);
    Eigen::VectorXcd EValues = es.eigenvalues();
    Eigen::VectorXd RealEValues = EValues.real();
    Eigen::MatrixXcd EVectors = es.eigenvectors();
    Eigen::MatrixXd RealEVectorsMatrix = EVectors.real();

    typedef std::vector<std::pair<double,Eigen::VectorXd> > DVPair;
    DVPair ToSort;

    for (int i=0; i<4; i++){
        ToSort.push_back(std::make_pair(RealEValues(i),RealEVectorsMatrix.block<4,1>(0,i)));
    }
    
    std::sort(ToSort.begin(),ToSort.end(),SortPairsFirstHighestToLowest());

    // For Outputting Ordered Eigenvalues and Vectors
    /*
    for (DVPair::iterator begin=ToSort.begin(); begin!=ToSort.end(); ++begin){
        cout << begin->first << endl;
        cout << begin->second << endl;
    }
    */
    
    return Eigen::Quaternion<double>(Eigen::Vector4d(ToSort[0].second));
}

/// @deprecated old aruns method, not as numerically stable as horn's method, which is preferred
Eigen::Matrix3d ArunsMethod(Eigen::Matrix3d Hsum)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hsum, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::Matrix3d R = svd.matrixV()*U.transpose();
    double det = R.determinant();
    // if (det == 1)
        return R;
    // write return some sort of error if not
}

/// Create transformation matrix from Rotation R and Translation vector P
Eigen::Matrix4d homogeneousmatrix(Eigen::Matrix3d R, Eigen::Vector3d p)
{
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F.block<3,3>(0,0) = R;
    F.block<3,1>(0,3) = p;
    return F;
}

/// Performs Horn Method of registration
///
/// @see http://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf
///
/// @param CloudA the first input cloud for which to find a transform
/// @param CloudB the second input cloud for which to find a transform
///
/// Both input clouds are expected to be 3 dimensions wide and n dimensions long
Eigen::Matrix4d hornRegistration(Eigen::MatrixXd CloudA, Eigen::MatrixXd CloudB)
{
	// average is calculated twice here
    Eigen::Vector3d abar=CloudA.colwise().mean();
    Eigen::Vector3d bbar=CloudB.colwise().mean();
    // subtract the average point coordinate from every
    // point position to center it on the origin
    // subtracts the average every point in x,y,z
    // this recenters the point cloud around 0
    CloudA.rowwise() -= abar.transpose();
    CloudB.rowwise() -= bbar.transpose();
    Eigen::Matrix3d H = Hmatrix(CloudA,CloudB);
    Eigen::Matrix4d G = Gmatrix(H);
    auto EV = EigenMatrix(G);
    Eigen::Matrix3d R = EV.toRotationMatrix();
    Eigen::Vector3d p = bbar-R*abar;
    Eigen::Matrix4d F = homogeneousmatrix(R,p);
    return F;
}

#endif // _HORN_REGISTRATION_HPP_