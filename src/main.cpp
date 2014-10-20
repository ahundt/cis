#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

Eigen::VectorXd vectorbar(Eigen::MatrixXd a)
{
    Eigen::VectorXd ax = a.col(0);
    Eigen::VectorXd ay = a.col(1);
    Eigen::VectorXd az = a.col(2);
    Eigen::Vector3d abar;
    abar(0) = ax.mean(); abar(1) = ay.mean(); abar(2) = az.mean();
    return abar;
}

Eigen::MatrixXd vectortilda(Eigen::MatrixXd a)
{
    Eigen::VectorXd abar = vectorbar(a);
    for(int j=0; j<3;j++)
       for(int i=0; i<a.rows();i++)
           a(i,j)=a(i,j)-abar(j);
    return a;
}

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

Eigen::Matrix3d Rmatrix(Eigen::Matrix3d Hsum)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Hsum, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::Matrix3d R = svd.matrixV()*U.transpose();
    double det = R.determinant();
    // if (det == 1)
        return R;
    // write return some sort of error if not
}

Eigen::Matrix4d homogeneousmatrix(Eigen::Matrix3d R, Eigen::Vector3d p)
{
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F.block<3,3>(0,0) = R;
    F.block<3,1>(0,3) = p;
    return F;
}


Eigen::Matrix4d frame(Eigen::MatrixXd a, Eigen::MatrixXd b)
{
    Eigen::Vector3d abar=vectorbar(a);
    Eigen::Vector3d bbar=vectorbar(b);
    Eigen::MatrixXd atilda=vectortilda(a);
    Eigen::MatrixXd btilda=vectortilda(b);
    Eigen::Matrix3d H = Hmatrix(atilda,btilda);
    Eigen::Matrix3d R = Rmatrix(H);
    Eigen::Vector3d p = bbar-R*abar;
    Eigen::Matrix4d F = homogeneousmatrix(R,p);
    return F;
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

    Eigen::Matrix4d F = frame(a,b);
    cout << F << endl;

}
