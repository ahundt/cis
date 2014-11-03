#ifndef CISHW1_H_INCLUDED
#define CISHW1_H_INCLUDED

/// Finds the maximum and minimum values of the point clouds in the X, Y,and Z
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
Eigen::MatrixXd FMatrix(double v)
{
    int N = 5;
    int index = 0;
    int columns = pow(N+1,3)-1;
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(1,columns);
    std::cout << "\n\nF is " << F << std::endl;
    for (int i=0; i<=5; i++){
        for(int j=0; j<=5; j++){
            for(int k=0; k<=5; k++){
                // Fijk = Bi * Bj * Bk F(0,index)
                std::cout << "\n\nindex is " << index << std::endl;
                double B = BersteinPolynomial(v,N,i)*BersteinPolynomial(v,N,j)*BersteinPolynomial(v,N,k);
                //std::cout << "\n\nB is " << B << std::endl;
                F.block<1,1>(0,index) << B;
                index++;
                BOOST_VERIFY(index<columns);
                //std::cout << "\n\nF is " << F << std::endl;
            }
        }
    }
    std::cout << "\n\ncolumns is " << columns << std::endl;
    return F;
}

#endif // CISHW1_H_INCLUDED
