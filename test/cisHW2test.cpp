#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CISHW1Test

// system includes
#include <boost/test/unit_test.hpp>
#include <exception>
#include <unistd.h>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include <boost/math/special_functions/binomial.hpp>

// local includes
#include "matrixOperations.hpp"
#include "PointData.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "hwDataConstants.hpp"
#include "DistortionCalibration.hpp"

static const bool debug = false;
static const double tolerance = 0.01l;

/// @todo The unit test requires a symlink from the executable folder to the location of the PA1-2 data folder, fix this using CMake.


template<typename T>
bool isWithinTolerance(const T& result, const T& groundTruth, double toleranceVal = tolerance){
    return (result.isApprox(groundTruth,toleranceVal) || (result - groundTruth).norm() < toleranceVal);
}

BOOST_AUTO_TEST_SUITE(cisPA2test)

BOOST_AUTO_TEST_CASE(BernsteinTest)
{

    // Testing
    double test = boost::math::binomial_coefficient<double>(3, 1);
    std::cout << "\n\nbinomial coefficient test is " << test << std::endl;
    double a=5.0;
    int b=3;
    int c=1;
    double Btest = BersteinPolynomial(a, b, c);
    std::cout << "\n\nBtest is " << Btest << std::endl;

    Eigen::MatrixXd test2(1,3);
    test2 << 0, 0.5, 1;
    Eigen::MatrixXd TestF = FMatrix(test2);
    std::cout << "\n\nF is " << TestF.transpose() << std::endl;
    //std::cout << "\n\nThe size of F is " << TestF.rows() << "x" << TestF.cols() <<std::endl;

    // 1-D Unit Test
    // A uniform distortion is applied to a set of points (y=x^2)
    // The expected result is that UndistortedUnitTest is about equal to the GroundTruth
    int max = 100;
    int min = 0;
    Eigen::VectorXd X = Eigen::VectorXd::LinSpaced(max+1,min,max);
    Y = X*X;
    Eigen::MatrixXd UnitTest = Eigen::Matrix::Zero(X.size,3);
    Eigen::MatrixXd GroundTruth = Eigen::Matrix::Zero(X.size,3);
    UnitTest.col(0) = Y;
    GroundTruth.col(0) = X;
    Eigen::MatrixXd cIJK = distortionCalibrationMatrixC(UnitTest, GroundTruth, Eigen::Vector3d minCorner, Eigen::Vector3d maxCorner);
    Eigen::MatrixXd Fmat = FMatrix(UnitTest);
    UnitTestUndistorted = Fmat*cIJK;
    BOOST_Verify(isWithinTolerance(UnitTestUndistorted,GroundTruth));

}


BOOST_AUTO_TEST_SUITE_END()
