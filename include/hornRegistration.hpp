#ifndef _HORN_REGISTRATION_HPP_
#define _HORN_REGISTRATION_HPP_

#include "matrixOperations.hpp"

/// creates Hmatrix for Horn's method
/// @see {Rigid3D3DCalculations.pdf slide 17's more detailed explanation alongside Arun's method
///      and slide 25 (horn's) for this step as part of Horn's Method}
Eigen::Matrix3d Hmatrix(Eigen::MatrixXd a, Eigen::MatrixXd b)
{
    BOOST_VERIFY(a.cols()==b.cols());
    Eigen::Matrix3d H;
    Eigen::Matrix3d Hsum = Eigen::Matrix3d::Zero(3,3);
    //std::cout << "\n\na.cols()="<<a.rows()<<"\n\n";
    for (int k=0; k<a.rows(); k++){
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++){
                H(i,j)=a(k,i)*b(k,j);
            }
        Hsum = Hsum+H;
    }
    return Hsum;
}

/// creates Gmatrix from Hmatrix using Horn's method
/// aka the Quaternion Method for Rotation R
/// @see Rigid3D3DCalculations.pdf slide 25
Eigen::Matrix4d Gmatrix(Eigen::Matrix3d H)
{
    Eigen::Vector3d delta;
    delta(0) = H(1,2)-H(2,1);
    delta(1) = H(2,0)-H(0,2);
    delta(2) = H(0,1)-H(1,0);
    Eigen::Matrix4d G;
    double traceH = H.trace();
    G(0,0)= traceH;
    G.block<3,1>(1,0) = delta;
    G.block<1,3>(0,1) = delta.transpose();
    G.block<3,3>(1,1) = H+H.transpose()-traceH*Eigen::Matrix3d::Identity();
    //std::cout << "\n\nG:\n\n" << G << "\n\n";
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

/// Need to find the largest eigenvalue and the
/// corresponding eigenvector of the Gmatrix.
/// The eigenvector corresponding to the largest
/// eigenvalue is the quaternion of the rotation matrix
///
/// @see Rigid3D3DCalculations.pdf slide 25
///
/// @return quaternion representing the rotation
Eigen::Quaternion<double> EigenMatrix(Eigen::Matrix4d G,bool debug = false)
{
    Eigen::EigenSolver<Eigen::Matrix4d> es(G);
    Eigen::VectorXcd EValues = es.eigenvalues();
    Eigen::VectorXd RealEValues = EValues.real();
    Eigen::MatrixXcd EVectors = es.eigenvectors();
    if(debug) std::cout << "\n\nEVectors:\n\n" << EVectors << "\n\n";
    Eigen::MatrixXd RealEVectorsMatrix = EVectors.real();

    typedef std::vector<std::pair<double,Eigen::VectorXd> > DVPair;
    DVPair ToSort;

    for (int i=0; i<4; i++){
        ToSort.push_back(std::make_pair(RealEValues(i),RealEVectorsMatrix.block<4,1>(0,i)));
    }

    std::sort(ToSort.begin(),ToSort.end(),SortPairsFirstHighestToLowest());

    // For Outputting Ordered Eigenvalues and Vectors
	if(debug){
		int i =0;
		for (DVPair::iterator begin=ToSort.begin(); begin!=ToSort.end(); ++begin){

			std::cout << "\n\nEval/Quat " << i << " :\n\n"
				<< begin->first << "\n\n"
					<< begin->second << "\n\n";
			++i;
		}
	}
    /// reorder for insertion into eigen @see http://eigen.tuxfamily.org/dox/classEigen_1_1Quaternion.html
    Eigen::Vector4d wxyz(ToSort[0].second);
    Eigen::Vector4d xyzw(wxyz[1],wxyz[2],wxyz[3],wxyz[0]);
	if(debug){
	    std::cout << "\n\nQuaternion:\n\n" << Eigen::Vector4d(ToSort[0].second)
	              << "\n\nQuatAsRot:\n\n"  << Eigen::Quaternion<double>(xyzw).toRotationMatrix()
	              << "\n\n";
	}
    return Eigen::Quaternion<double>(xyzw);
}

/// @deprecated old aruns method, not as numerically stable as horn's method, which is preferred
Eigen::Matrix3d ArunsMethod(Eigen::Matrix3d Hsum)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hsum, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::Matrix3d R = svd.matrixV()*U.transpose();
    double det = R.determinant();
    if (det == -1) throw std::runtime_error("WARNING: Reflection of Points Rather than Rigid Body Motion, Det == -1");
    return R;
}

/// Create transformation matrix from Rotation R and Translation vector P
Eigen::Matrix4d homogeneousmatrix(Eigen::Matrix3d R, Eigen::Vector3d p)
{
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F.block<3,3>(0,0) = R;
    F.block<3,1>(0,3) = p;
    return F;
}

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

/// Performs Horn Method of registration
///
/// @see http://people.csail.mit.edu/bkph/papers/Absolute_Orientation.pdf
///
/// @param CloudA the first input cloud for which to find a transform
/// @param CloudB the second input cloud for which to find a transform
///
/// Both input clouds are expected to be 3 dimensions wide and n dimensions long
Eigen::Matrix4d hornRegistration(const Eigen::MatrixXd& CloudA, const Eigen::MatrixXd& CloudB, bool debug = false)
{
	// average the point clouds
    Eigen::Vector3d abar=CloudA.colwise().mean();
    Eigen::Vector3d bbar=CloudB.colwise().mean();
    // leave out but still compile this code
	if(debug) {
	    std::cout << "\n\nabar:\n\n" << abar
	              << "\n\nbbar:\n\n" << bbar
	              << "\n\nCloudA:\n\n" << CloudA
	              << "\n\nCloudB:\n\n" << CloudB
	              << "\n\n";
	}
    // subtract the average point coordinate from every
    // point position to center it on the origin
    // subtracts the average every point in x,y,z
    // this recenters the point cloud around 0
    // @see Rigid3D3DCalculations.pdf slide 4 for this step
    Eigen::MatrixXd CloudAMoved = CloudA;
    Eigen::MatrixXd CloudBMoved = CloudB;
    CloudAMoved.rowwise() -= abar.transpose();
    CloudBMoved.rowwise() -= bbar.transpose();
    Eigen::Matrix3d H = Hmatrix(CloudAMoved,CloudBMoved);
    Eigen::Matrix4d G = Gmatrix(H);
    auto EV = EigenMatrix(G);
    Eigen::Matrix3d R = EV.toRotationMatrix();

	if(debug) {
	    std::cout << "\n\nR:\n\n" << R
	              << "\n\n";
	}
    Eigen::Vector3d p = bbar-R*abar;
    Eigen::Matrix4d F = homogeneousmatrix(R,p);
    return F;
}

#endif // _HORN_REGISTRATION_HPP_
