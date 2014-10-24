#include <iostream>
#include <vector>
#include <boost/bind.hpp>


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
struct comparePairs {
template <typename K, typename V>
bool operator()(const std::pair<K,V>& lhs, const std::pair<K,V>& rhs)
{
  return lhs.first < rhs.first;
}
};

/// Need to find the largest eigenvalue and the corresponding eigenvector
/// of the Gmatrix.
Eigen::Matrix4d EigenMatrix(Eigen::Matrix4d G)
{
    Eigen::EigenSolver<Eigen::Matrix4d> es(G);
    Eigen::Vector4cd EValues = es.eigenvalues();
    Eigen::Vector4d RealEValues = EValues.real();
    Eigen::Matrix4cd EVectors = es.eigenvectors();
    Eigen::Matrix4d RealEVectors = EVectors.real();


    std::vector<pair<double,Eigen::Vector4d> > ToSort;
    for (int i=0; i<4; i++){
        ToSort.push_back(std::make_pair(RealEValues(i),RealEVectors.block<4,1>(0,i)));
    }
    /*
    std::sort(ToSort.begin(),ToSort.end(),comparePairs());
    cout << RealEValues << endl << es.eigenvectors() << endl << "Test" << endl;
    //Eigen::ComplexEigenSolver<Eigen::MatrixXcf> ces(G);
    //ces.compute(G);
    //cout << ces.eigenvector(G) << endl;
    //G.compute();
    //return es.eigenvectors();
    */
}

/// Function for converting unit quaternion to a rotation
/// @todo replace with built in eigen call
Eigen::Matrix3d UnitQuaternionToRotation(Eigen::Vector4d q)
{
    Eigen::Matrix3d R;
    R(0,0) = q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3);
    R(0,1) = 2*(q(1)*q(2)-q(0)*q(3));
    R(0,2) = 2*(q(1)*q(3)+q(0)*q(2));
    R(1,0) = 2*(q(1)*q(2)+q(0)*q(3));
    R(1,1) = q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3);
    R(1,2) = 2*(q(2)*q(3)-q(0)*q(1));
    R(2,0) = 2*(q(1)*q(3)-q(0)*q(2));
    R(2,1) = 2*(q(2)*q(3)+q(0)*q(1));
    R(2,2) = q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);
    return R;
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

/// combines above methods
Eigen::Matrix4d frame(Eigen::MatrixXd a, Eigen::MatrixXd b)
{
	// average is calculated twice here
    Eigen::Vector3d abar=vectorbar(a);
    Eigen::Vector3d bbar=vectorbar(b);
    Eigen::MatrixXd atilda=centerPointsOnOrigin(a);
    Eigen::MatrixXd btilda=centerPointsOnOrigin(b);
    Eigen::Matrix3d H = Hmatrix(atilda,btilda);

    // Add H to FindLargestEigenVector Function
    // Add EigenVector to Quaternion to RMatrix Function

    // Eigen::Matrix3d R = Rmatrix(H);

    // Eigen::Vector3d p = bbar-R*abar;
    // Eigen::Matrix4d F = homogeneousmatrix(R,p);
    // return F;
}

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

    Eigen::Matrix3d H = Hmatrix(a,b);
    Eigen::Matrix4d G = Gmatrix(H);
    EigenMatrix(G);

    //int cols = 4;
    //int rows = 4;
    //Eigen::MatrixXf G(rows, 2*cols);
    //Eigen::MatrixXcf X(rows, cols);
    //X.real() = G;//.leftCols(cols);
    // X.imag() = G.rightCols(cols);

    // Eigen::Matrix4d F = frame(a,b);
    cout << H << endl << G << endl;

}
