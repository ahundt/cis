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

template<typename T>
void PrintTwo(T& tp1, T& tp2, std::string p1 = std::string("p1:"), std::string p2 = std::string("p2:")){
    
    std::cout << "\n\n"<< p1 <<"\n\n" << tp1 << "\n\n"<< p2 <<"\n\n" << tp2 <<"\n\n";
}

struct checkHornRegistrationInverses{
    void operator()(const Eigen::MatrixXd& trackerPerspective1, const Eigen::MatrixXd& trackerPerspective2, std::string p1 = std::string("p1:"), std::string p2 = std::string("p2:")){
        
        Eigen::Matrix4d F = hornRegistration(trackerPerspective1,trackerPerspective2);
        Eigen::Matrix4d Finv = hornRegistration(trackerPerspective2,trackerPerspective1);
    
        BOOST_CHECK(Eigen::Matrix4d::Identity().isApprox(F*Finv,tolerance));
        if(!Eigen::Matrix4d::Identity().isApprox(F*Finv,tolerance)){
            std::cout << "\n\n"<< p1 <<"\n\n" << trackerPerspective1 << "\n\n"<< p2 <<"\n\n" << trackerPerspective2  << "\n\nF:\n\n" << F << "\n\nFinv:\n\n" << Finv << "\n\nF*Finv:\n\n" << F*Finv <<"\n\n";
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

BOOST_AUTO_TEST_SUITE(VariantsSuite)

BOOST_AUTO_TEST_CASE(simplePass)
{
}

BOOST_AUTO_TEST_CASE(HMatrixGMatrix)
{
    // Test HMatrix and GMatrix functions
    
    // test 1
    
    Eigen::MatrixXd a(1,3);
    a << 1,0,0;
    
    Eigen::MatrixXd b(1,3);
    b << 1,0,0;
    
    Eigen::Matrix3d H = Hmatrix(a,b);
    Eigen::Matrix3d Hexpected;
    Hexpected <<
    1,0,0,
    0,0,0,
    0,0,0;
    
    Eigen::Matrix4d G = Gmatrix(H);
    Eigen::Matrix4d Gexpected;
    Gexpected <<
    1,  0,  0 ,  0,
    0,  1,  0 ,  0,
    0,  0, -1 ,  0,
    0,  0,  0 , -1;
    
    BOOST_CHECK_EQUAL(Hexpected, H);
    BOOST_CHECK_EQUAL(Gexpected, G);
    
    
    // test 2
    
    
    a << 1,1,1;
    b << 1,1,1;
    H = Hmatrix(a,b);
    Hexpected <<
    1,1,1,
    1,1,1,
    1,1,1;
    
    G = Gmatrix(H);
    Gexpected <<
    3,  0,  0 ,  0,
    0, -1,  2 ,  2,
    0,  2, -1 ,  2,
    0,  2,  2 , -1;
    
    BOOST_CHECK_EQUAL(Hexpected, H);
    BOOST_CHECK_EQUAL(Gexpected, G);
    
    
    
    // test 3
    
    a << 1,0,2;
    b << 1,0,2;
    H = Hmatrix(a,b);
    Hexpected <<
    1,0,2,
    0,0,0,
    2,0,4;
    
    
    G = Gmatrix(H);
    Gexpected <<
    5,  0,  0 ,  0,
    0, -3,  0 ,  4,
    0,  0, -5 ,  0,
    0,  4,  0 ,  3;
    
    BOOST_CHECK_EQUAL(Hexpected, H);
    BOOST_CHECK_EQUAL(Gexpected, G);
    
    
    // test 4
    
    a.resize(2,3);
    a << 1,0,2,
         1,1,1;
    b.resize(2, 3);
    b << 1,0,2,
         1,1,1;
    
    H = Hmatrix(a,b);
    Hexpected <<
    2,1,3,
    1,1,1,
    3,1,5;
    
    G = Gmatrix(H);
    Gexpected <<
    8,  0,  0 ,  0,
    0, -4,  2 ,  6,
    0,  2, -6 ,  2,
    0,  6,  2 ,  2;
    
    BOOST_CHECK_EQUAL(Hexpected, H);
    BOOST_CHECK_EQUAL(Gexpected, G);

}

#if 0
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
#endif
BOOST_AUTO_TEST_CASE(manualHornRegistration)
{
    
    
    Eigen::MatrixXd trackerPoints(8,3);
	trackerPoints <<
  		  0,   0,   0,
  		  0,   0, 250,
  		  0, 250,   0,
  		  0, 250, 250,
  		250,   0,   0,
  		250,   0, 250,
  		250, 250,   0,
  		250, 250, 250;
		

     std::cout << "\n\ntrackerPoints:\n\n" << trackerPoints << std::endl;
	 
   	 Eigen::Matrix4d manualTform;
   	 manualTform <<
   	 1,  0,  0,    1,
   	 0,  1,  0,    1,
   	 0,  0,  1,    1,
   	 0,  0,  0,    1;
	 
	 
	 Eigen::MatrixXd trackerPointsManTform = trackerPoints;
	 
	 trackerPointsManTform.rowwise() += Eigen::Vector3d(1,1,1).transpose();
    
    std::cout << "\n\ntrackerPointsManTform:\n\n" << trackerPointsManTform << std::endl;

     Eigen::Matrix4d hornTform = hornRegistration(trackerPoints,trackerPointsManTform);
	 
     BOOST_CHECK(manualTform.isApprox(hornTform,tolerance));
    PrintTwo(manualTform, hornTform,"manualTform:","hornTform:");
    
    std::cout << "\n\n <<<<<<beg<<<<<<<<< \n\n";
    checkHornRegistrationInverses()(trackerPoints,trackerPointsManTform,"trackerPoints","trackerPointsManTform");
    std::cout << "\n\n <<<<<<end<<<<<<<<< \n\n";
	
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