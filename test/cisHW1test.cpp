#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE CISHW1Test
#include <boost/test/unit_test.hpp>
#include <exception>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <boost/bind.hpp>
#include "matrixOperations.hpp"
#include "hornRegistration.hpp"

int add(int i, int j)
{
    return i + j;
}

BOOST_AUTO_TEST_SUITE(VariantsSuite)

BOOST_AUTO_TEST_CASE(simplePass)
{
}

BOOST_AUTO_TEST_CASE(basicHornRegistration)
{

    Eigen::Matrix3d a;
     a << 1, 2, 3,
      2, 4, 6,
      3, 6, 9;

      Eigen::Matrix3d b;
     b << 1, 2, 3,
      4, 5, 6,
      5, 8, 9;

     Eigen::Matrix4d hornTform = hornRegistration(a,b);
     //std::cout << F << std::endl;
	 
	 Eigen::Matrix4d manualTform;
	 manualTform <<
	 0.970314 , -0.165993 ,  0.175889 ,   1.00135 ,
	 -0.175889,  -0.983508,  0.0421373,    9.03298,
	  0.165993, -0.0718232,  -0.983508,    11.8564,
	         0,          0,          0,          1;
	 
    BOOST_CHECK(hornTform.isApprox(manualTform,.01));
}

BOOST_AUTO_TEST_CASE(multipleCheckFailures)
{
    BOOST_CHECK(add(2, 2) == 4);
    BOOST_CHECK(add(2, 2) == 4);
    BOOST_CHECK(add(2, 2) == 4);
}

BOOST_AUTO_TEST_CASE(requireFailure)
{
    BOOST_REQUIRE(add(2, 2) == 4);
}

BOOST_AUTO_TEST_CASE(explicitError)
{
    // BOOST_ERROR("Error message");
}

BOOST_AUTO_TEST_CASE(explicitFailure)
{
    // BOOST_FAIL("Failure message");
}

BOOST_AUTO_TEST_CASE(errorThenFailure)
{
    // BOOST_FAIL("Error message");
    // BOOST_FAIL("Failure message");
}

BOOST_AUTO_TEST_CASE(uncaughtException)
{
    // throw "Catch me if you can!";
}

BOOST_AUTO_TEST_CASE(stdException)
{
    // throw new std::exception();
}

BOOST_AUTO_TEST_CASE(checkMessageFailure)
{
     BOOST_CHECK_MESSAGE(add(2, 2) == 4, "add(..) result: " << add(2, 2));
}

BOOST_AUTO_TEST_CASE(checkEqualFailure)
{
    BOOST_CHECK_EQUAL(add( 2,2 ), 4);
}

BOOST_AUTO_TEST_CASE(threeSeconds)
{
    // sleep(3);
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE(PassingSuite)

BOOST_AUTO_TEST_CASE(pass1)
{
}

BOOST_AUTO_TEST_CASE(pass2)
{
}

BOOST_AUTO_TEST_CASE(pass3)
{
}

BOOST_AUTO_TEST_SUITE_END()