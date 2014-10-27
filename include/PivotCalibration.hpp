#ifndef _PIVOT_CALIBRATION_HPP_
#define _PIVOT_CALIBRATION_HPP_

Eigen::MatrixXd makeAmatrix(Eigen::MatrixXd F)
{
    Eigen::MatrixXd A(3,6);
    A.block<3,3>(0,0) = F.block<3,3>(0,0);
    A.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
    return A;
}

Eigen::MatrixXd makePVector(Eigen::MatrixXd F)
{
    Eigen::MatrixXd P(3,1);
    P.block<3,1>(0,0) = F.block<3,1>(0,3);
    return P;
}

Eigen::VectorXd SVDSolve(Eigen::MatrixXd RI, Eigen::VectorXd p)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(RI, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd X = svd.solve(-p);
    return X;
}


#endif // _PIVOT_CALIBRATION_HPP_
