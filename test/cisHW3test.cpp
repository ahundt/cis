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
#include "parsePA1_2.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "PA1_2_DataConstants.hpp"
#include "DistortionCalibration.hpp"
#include "ICP.hpp"


static const bool debug = false;

/// @todo The unit test requires a symlink from the executable folder to the location of the PA1-2 data folder, fix this using CMake.


BOOST_AUTO_TEST_SUITE(cisPA3test)

BOOST_AUTO_TEST_CASE(FindClosestPointPointTest)
{
	// Unit Tests to Check if the function FindClosestPoint() works correctly
	Eigen::Vector3d point1(0,1,0);
    Eigen::Vector3d point2(1,0,0);
    Eigen::Vector3d point3(0,0,0);
    std::vector<Eigen::Vector3d> triangle(3);
    triangle[1] = point1;
    triangle[2] = point2;
    triangle[3] = point3;
    Eigen::Vector3d testpoint1(1,1,0);
    Eigen::Vector3d testpoint2(0.5,-1,0);
    Eigen::Vector3d testpoint3(0.5,0.5,1);
    Eigen::Vector3d testpoint4(-1,0.5,0);
    Eigen::Vector3d testpoint5(0,0,0);
    Eigen::Vector3d testpoint6(0.2,0.2,1);
    Eigen::Vector3d testpoint7(-1,-1,-1);
    Eigen::Vector3d testpoint8(2,0,0);

    Eigen::Vector3d testresult1 = FindClosestPoint(testpoint1,triangle);
    Eigen::Vector3d testresult2 = FindClosestPoint(testpoint2,triangle);
    Eigen::Vector3d testresult3 = FindClosestPoint(testpoint3,triangle);
    Eigen::Vector3d testresult4 = FindClosestPoint(testpoint4,triangle);
    Eigen::Vector3d testresult5 = FindClosestPoint(testpoint5,triangle);
    Eigen::Vector3d testresult6 = FindClosestPoint(testpoint6,triangle);
    Eigen::Vector3d testresult7 = FindClosestPoint(testpoint7,triangle);
    Eigen::Vector3d testresult8 = FindClosestPoint(testpoint8,triangle);

    std::cout << "\nTest result 1: \n" << testresult1 << std::endl << std::endl;
    std::cout << "\nTest result 2: \n" << testresult2 << std::endl << std::endl;
    std::cout << "\nTest result 3: \n" << testresult3 << std::endl << std::endl;
    std::cout << "\nTest result 4: \n" << testresult4 << std::endl << std::endl;
    std::cout << "\nTest result 5: \n" << testresult5 << std::endl << std::endl;
    std::cout << "\nTest result 6: \n" << testresult6 << std::endl << std::endl;
    std::cout << "\nTest result 7: \n" << testresult7 << std::endl << std::endl;
    std::cout << "\nTest result 8: \n" << testresult8 << std::endl << std::endl;
}
    

BOOST_AUTO_TEST_SUITE_END()
