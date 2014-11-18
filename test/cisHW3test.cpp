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
#include <Eigen/Core>
#include <Eigen/Dense>

// local includes
#include "ICP.hpp"


static const bool debug = false;
static const double tolerance = 0.01l;

/// @todo The unit test requires a symlink from the executable folder to the location of the PA1-2 data folder, fix this using CMake.

template<typename T>
bool isWithinTolerance(const T& result, const T& groundTruth, double toleranceVal = tolerance){
    return (result.isApprox(groundTruth,toleranceVal) || (result - groundTruth).norm() < toleranceVal);
}

BOOST_AUTO_TEST_SUITE(cisPA3test)

void testClosestPoint(const Eigen::Vector3d&& point,std::vector<Eigen::Vector3d>& vertices, Eigen::VectorXd& triangle, const Eigen::Vector3d&& expectedResult){
    
    Eigen::Vector3d testresult = FindClosestPoint(point,vertices,triangle);
    
    if (!isWithinTolerance(testresult, expectedResult)) {
        std::cout << "error: input point:" << point.transpose() << " expectedResult: " << expectedResult.transpose() << " not equal to testresult: " << testresult.transpose() << "\n";
    }
    BOOST_CHECK(isWithinTolerance(testresult, expectedResult));
}

BOOST_AUTO_TEST_CASE(FindClosestPointPointTest)
{
	// Unit Tests to Check if the function FindClosestPoint() works correctly
    Eigen::VectorXd triangle(3);
    triangle(0) = 0;
    triangle(1) = 1;
    triangle(2) = 2;
	
	std::vector<Eigen::Vector3d> vertices;
	vertices.push_back(Eigen::Vector3d(0,1,0));
    vertices.push_back(Eigen::Vector3d(1,0,0));
    vertices.push_back(Eigen::Vector3d(0,0,0)); 

    testClosestPoint(Eigen::Vector3d(1,1,0)    , vertices, triangle, Eigen::Vector3d(  0.5   , 0.5    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(0.5,-1,0) , vertices, triangle, Eigen::Vector3d(  0.5   , 0.0    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(0.5,0.5,1), vertices, triangle, Eigen::Vector3d(  0.5   , 0.5    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(-1,0.5,0) , vertices, triangle, Eigen::Vector3d(  0.0   , 0.5    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(0,0,0)    , vertices, triangle, Eigen::Vector3d(  0.0   , 0.0    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(0.2,0.2,1), vertices, triangle, Eigen::Vector3d(  0.2   , 0.2    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(-1,-1,-1) , vertices, triangle, Eigen::Vector3d(  0.0   , 0.0    , 0.0 ));
    testClosestPoint(Eigen::Vector3d(2,0,0)    , vertices, triangle, Eigen::Vector3d(  1.0   , 0.0    , 0.0 ));
}
    

BOOST_AUTO_TEST_SUITE_END()
