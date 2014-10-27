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
#include "PointData.hpp"

static const double tolerance = 0.01l;

static const std::string relativeDataPath("PA1-2/");
static const std::string pa1debuga("pa1-debug-a-");
static const std::string pa1debugb("pa1-debug-b-");
static const std::string pa1debugc("pa1-debug-c-");
static const std::string pa1debugd("pa1-debug-d-");
static const std::string pa1debuge("pa1-debug-e-");
static const std::string pa1debugf("pa1-debug-f-");
static const std::string pa1debugg("pa1-debug-g-");

/// @todo The unit test requires a symlink from the executable folder to the location of the PA1-2 data folder, fix this using CMake.

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
	 
    BOOST_CHECK(hornTform.isApprox(manualTform,tolerance));
}

struct checkHornRegistrationInverses{
void operator()(const Eigen::MatrixXd& trackerPerspective1, const Eigen::MatrixXd& trackerPerspective2){
    
    Eigen::Matrix4d F = hornRegistration(trackerPerspective1,trackerPerspective2);
    Eigen::Matrix4d Finv = hornRegistration(trackerPerspective2,trackerPerspective1);
    
    BOOST_CHECK(Eigen::Matrix4d::Identity().isApprox(F*Finv,tolerance));
    if(!Eigen::Matrix4d::Identity().isApprox(F*Finv,tolerance)){
        std::cout << "\n\np1:\n\n" << trackerPerspective1 << "\n\np2:\n\n" << trackerPerspective2  << "\n\nF:\n\n" << F << "\n\nFinv:\n\n" << Finv << "\n\nF*Finv:\n\n" << F*Finv <<"\n\n";
    }
}
};


template<typename Visitor>
void visitEachTracker(const csvCIS_pointCloudData::TrackerFrames& tf1,const csvCIS_pointCloudData::TrackerFrames& tf2,Visitor v){
    
    auto frameIt1 = tf1.begin();
    auto frameIt2 = tf2.begin();
    for(; frameIt1 != tf1.end() && frameIt2 != tf2.end(); ++frameIt1, ++frameIt2)
    {
        auto trackerIt3 = frameIt1->begin();
        auto trackerIt4 = frameIt2->begin();
        for(; trackerIt3 != frameIt1->end() && trackerIt4 != frameIt2->end(); ++trackerIt3, ++trackerIt4)
        {
            v(*trackerIt3,*trackerIt4);
        }
    }
    
}


/// useful for going over calbody and calreadings, where there is one instance of calbody and many instances of calreadings
template<typename Visitor>
void visitSecondTrackerRepeatedly(const csvCIS_pointCloudData::TrackerFrames& tf1,const csvCIS_pointCloudData::TrackerFrames& tf2,Visitor v){
    
    for(auto frameIt1 = tf1.begin(); frameIt1 != tf1.end(); ++frameIt1)
    {
        for(auto frameIt2 = tf2.begin(); frameIt2 != tf2.end(); ++frameIt2)
        {
            auto trackerIt3 = frameIt1->begin();
            auto trackerIt4 = frameIt2->begin();
            for(; trackerIt3 != frameIt1->end() && trackerIt4 != frameIt2->end(); ++trackerIt3, ++trackerIt4)
            {
                v(*trackerIt3,*trackerIt4);
            }
        }
    }
    
}


BOOST_AUTO_TEST_CASE(testDebugData)
{
    AlgorithmData ad(assembleHW1AlgorithmData(relativeDataPath,pa1debuga));
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
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