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

// local includes
#include "matrixOperations.hpp"
#include "hornRegistration.hpp"
#include "PointData.hpp"
#include "PivotCalibration.hpp"
#include "PointEstimation.hpp"
#include "hw1Constants.hpp"

static const bool debug = false;
static const double tolerance = 0.01l;

/// @todo The unit test requires a symlink from the executable folder to the location of the PA1-2 data folder, fix this using CMake.

int add(int i, int j)
{
    return i + j;
}


template<typename T>
bool isWithinTolerance(const T& result, const T& groundTruth, double toleranceVal = tolerance){
    return (result.isApprox(groundTruth,toleranceVal) || (result - groundTruth).norm() < toleranceVal);
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
       // if(!Eigen::Matrix4d::Identity().isApprox(F*Finv,tolerance)){
            std::cout << "\n\n"<< p1 <<"\n\n" << trackerPerspective1 << "\n\n"<< p2 <<"\n\n" << trackerPerspective2  << "\n\nF:\n\n" << F << "\n\nFinv:\n\n" << Finv << "\n\nF*Finv:\n\n" << F*Finv <<"\n\n";
       // }
    }
};

struct getHornRegistrationMatrix{
    getHornRegistrationMatrix():m_matrixvectorP(new std::vector<Eigen::MatrixXd>()){}
    void operator()(const Eigen::MatrixXd& trackerPerspective1, const Eigen::MatrixXd& trackerPerspective2){
        Eigen::Matrix4d F = hornRegistration(trackerPerspective1,trackerPerspective2);
        m_matrixvectorP->push_back(Eigen::MatrixXd(F));
    }
    boost::shared_ptr<std::vector<Eigen::MatrixXd>> m_matrixvectorP;
};

/// visits both trackers simultaneously, if either reaches the end it moves on to the next section
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

#if 0
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

BOOST_AUTO_TEST_CASE(manualHornRegistration)
{
    static const bool debug = false;

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


    if(debug) std::cout << "\n\ntrackerPoints:\n\n" << trackerPoints << std::endl;

   	Eigen::Matrix4d manualTform;
   	manualTform <<
   	1,  0,  0,    1,
   	0,  1,  0,    1,
   	0,  0,  1,    1,
   	0,  0,  0,    1;


    Eigen::MatrixXd trackerPointsManTform = trackerPoints;

    trackerPointsManTform.rowwise() += Eigen::Vector3d(1,1,1).transpose();

    if(debug) std::cout << "\n\ntrackerPointsManTform:\n\n" << trackerPointsManTform << std::endl;

    Eigen::Matrix4d hornTform = hornRegistration(trackerPoints,trackerPointsManTform);

    BOOST_CHECK(manualTform.isApprox(hornTform,tolerance));
    if(debug) PrintTwo(manualTform, hornTform,"manualTform:","hornTform:");

    checkHornRegistrationInverses()(trackerPoints,trackerPointsManTform,"trackerPoints","trackerPointsManTform");

}
#endif
BOOST_AUTO_TEST_CASE(testLittleG_EMPivotCalibration)
{
    AlgorithmData ad;
    
    // a
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugg);
    //visitEachTracker(ad..frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.empivot.frames, ad.empivot.frames, checkHornRegistrationInverses());
}
#if 0
BOOST_AUTO_TEST_CASE(testDebugData)
{
    AlgorithmData ad;

    // a
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debuga);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // b
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugb);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // c
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugc);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // d
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugd);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // e
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debuge);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // f
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugf);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());

    // g
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debugg);
    visitEachTracker(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
    visitSecondTrackerRepeatedly(ad.calbody.frames, ad.calreadings.frames, checkHornRegistrationInverses());
}

BOOST_AUTO_TEST_CASE(solveForCExpected)
{
    AlgorithmData ad;
    ad = assembleHW1AlgorithmData(relativeDataPath,pa1debuga);

    // a
	std::vector<Eigen::MatrixXd> cExpected = estimateCExpected(ad.calreadings.frames,ad.calbody.frames,debug);


    if(debug){
        std::cout << "\n\nsolveForCExpected\n\n" << cExpected.size() << "\n\n";
        for (int i = 0; i<cExpected.size(); i++) std::cout << cExpected[i] << "\n\n";
    }
}

void testOnePivotCalibration(csvCIS_pointCloudData::TrackerDevices trackerIndexedData, Eigen::Vector3d checkOutput, std::string description = "", bool debug = false) {
    Eigen::VectorXd result = pivotCalibration(trackerIndexedData,debug);
    Eigen::Vector3d checkResultFirst = result.block<3,1>(0,0);
    Eigen::Vector3d checkResultSecond = result.block<3,1>(3,0);

    // note: isApprox is known to break near zero
    BOOST_CHECK(isWithinTolerance(checkResultSecond,checkOutput));

    if (!isWithinTolerance(checkResultSecond,checkOutput)) {
        std::cout << "\n\n" << description << "\n\n";
        std::cout << "===============================================\n\n";
        std::cout << "\n\nresult:\n\n" << result << "\n\n";
        std::cout << "\n\ncheckresult FULL:\n\n" << result << "\n\n";
        std::cout << "\n\ncheckresult FIRST:\n\n" << checkResultFirst << "\n\ncheckresult SECOND:\n\n" << checkResultSecond << "\n\ncheckoutput:\n\n" << checkOutput << "\n\n";
    }
};

void testOnePivotCalibration(std::string relativeDataPath,std::string datapathsuffix, bool debug = false){

    if (debug) {
        std::cout << "testing " <<relativeDataPath<<datapathsuffix<<"\n";
        std::cout << "===============================================\n\n";
    }

    AlgorithmData ad;
    csvCIS_pointCloudData::TrackerDevices trackerIndexedData;

    // Note: we know there is only one tracker in this data
    //       so we can run concat to combine the vectors and
    //       and do the calibration for it.

    // a
    ad = assembleHW1AlgorithmData(relativeDataPath,datapathsuffix);
    trackerIndexedData = concat(ad.empivot.frames);

    Eigen::Vector3d checkOutput = ad.output1.estElectromagneticPostPos;

    testOnePivotCalibration(trackerIndexedData, checkOutput, datapathsuffix,debug);

}

void testTwoPivotCalibration(std::string relativeDataPath,std::string datapathsuffix, bool debug = false){


    AlgorithmData ad;
    csvCIS_pointCloudData::TrackerFrames trackerIndexedData;

    ad = assembleHW1AlgorithmData(relativeDataPath,datapathsuffix);
    trackerIndexedData = swapIndexing(ad.optpivot.frames);
    /// @todo contains two different ways of attempting to do this worked on during debugging
    /// @todo The core functions are likely correct, but we believe that we are using one of the wrong frames, the wrong calibration object, or the wrong ordering and that is causing the offset. We expect a data sourcing and frame transform ordering error rather than a flaw in the underlying algorithms.
    Eigen::VectorXd optProbePivotPtAndCalInOptCoord = pivotCalibration(trackerIndexedData[1],debug);
    if(debug) std::cout << "\n\ntestTwoPivot - p:\n\n" << optProbePivotPtAndCalInOptCoord << "\n\n";
    Eigen::Vector3d optProbeCalInOptCoord = optProbePivotPtAndCalInOptCoord.block<3,1>(0,0);
    Eigen::Vector3d optProbePivotPtInOptCoord = optProbePivotPtAndCalInOptCoord.block<3,1>(3,0);

    Eigen::VectorXd emTrackerPivotPtAndCalInOptCoord = pivotCalibration(trackerIndexedData[0],debug);
    Eigen::Vector3d emCalInOptCoord = emTrackerPivotPtAndCalInOptCoord.block<3,1>(0,0);
    Eigen::Vector3d emTrackerPivotPtInOptCoord = emTrackerPivotPtAndCalInOptCoord.block<3,1>(3,0);
    
    
    Eigen::VectorXd result = pivotCalibrationTwoSystems(trackerIndexedData[0],trackerIndexedData[1],debug);


    Eigen::MatrixXd dcloud = registrationToFirstCloud(trackerIndexedData[0]);

    Eigen::MatrixXd hcloud = registrationToFirstCloud(trackerIndexedData[1]);
//
//    Eigen::Affine3d FDinv;
//    FDinv.matrix() = homogeneousInverse(FD);
//    FDinv.matrix() = Eigen::Matrix4d(FD.block<4,4>(0,0));
//
//    Eigen::Vector3d optPivotPtInEMCoord = FDinv*optProbePivotPtInOptCoord;
    
    // skip tranform into other coords for now
    Eigen::Vector3d optPivotPtInEMCoord = optProbePivotPtInOptCoord;

    Eigen::Vector3d groundTruthOptPivotPtInEMCoord = ad.output1.estOpticalPostPos;

    
    bool isWithinTolerance_ = isWithinTolerance(optPivotPtInEMCoord,groundTruthOptPivotPtInEMCoord);
    
    if (debug || !isWithinTolerance_) {
        std::cout << "testing " <<relativeDataPath<<datapathsuffix<<"\n";
        std::cout << "===============================================\n\n";
    }
    
    
    BOOST_CHECK(isWithinTolerance(optPivotPtInEMCoord,groundTruthOptPivotPtInEMCoord));
    
    if(debug || !isWithinTolerance_){
        Print(trackerIndexedData[0],true,"emTrackerIndexedData0"); // em tracker
        Print(trackerIndexedData[1],true,"opticalProbeIndexedData1"); // optical probe
    }
    if (!isWithinTolerance_) {
        std::cout << "\n\noptProbePivotPtInOptCoord:\n\n" << optProbePivotPtInOptCoord << "\n\n";
        std::cout << "\n\ngroundTruthOptPivotPtInEMCoord:\n\n" << groundTruthOptPivotPtInEMCoord << "\n\n";
        std::cout << "\n\nemTrackerPivotPtInOptCoord:\n\n" << emTrackerPivotPtInOptCoord << "\n\n";
        //std::cout << "\n\FD:\n\n" << FD << "\n\n";
        //std::cout << "\n\FH:\n\n" << FH << "\n\n";
    }
};

// Make a custom cloud with this pattern,
// 4 v shapes of 5 points centered around 0,0
// all pointing towards the center.
// This is to be used and transformed as a test
// of the calibration algorihtms.
//
//              \  |  /
//             \ \ | / /
//              \ \|/ /
//               \ | /
//    -------------|-------------
//               / | \
//              / /|\ \
//             / / | \ \
//              /  |  \
//

csvCIS_pointCloudData::TrackerDevices makeCustomClouds(){

    csvCIS_pointCloudData::TrackerDevices clouds;

    Eigen::MatrixXd probePointsTest(5,3);
    probePointsTest <<
    0,   1,   0,
    1,   2,   0,
    -1,   2,   0,
    2,   3,   0,
    -2,   3,   0;

    clouds.push_back(probePointsTest);

    probePointsTest <<
    1,   0,   0,
    2,   1,   0,
    2,  -1,   0,
    3,   2,   0,
    3,  -2,   0;

    clouds.push_back(probePointsTest);

    probePointsTest <<
    0,  -1,   0,
    1,  -2,   0,
    -1,  -2,   0,
    2,  -3,   0,
    -2,  -3,   0;

    clouds.push_back(probePointsTest);

    probePointsTest <<
    -1,   0,   0,
    -2,   1,   0,
    -2,  -1,   0,
    -3,   2,   0,
    -3,  -2,   0;

    clouds.push_back(probePointsTest);

    return clouds;
}

BOOST_AUTO_TEST_CASE(PivotCalibration){

    csvCIS_pointCloudData::TrackerDevices clouds = makeCustomClouds();


    //Print(clouds,true, "clouds1");

    testOnePivotCalibration(clouds, Eigen::Vector3d(0,0,0), "manualAtZero");


    ////////////////////////////////////////////////////
    // transform clouds for a translation test
    Eigen::Affine3d transform;

    transform.setIdentity();

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 2.5, 0.0, 0.0;

    double theta = 0;//boost::math::constants::pi<double>();
    // The same rotation matrix as before; tetha radians arround Z axis
    transform.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

    for (auto&& tracker : clouds) {
        for(int i = 0; i < tracker.rows(); i++){
            // translate all of the points using an Homogenous transform matrix
            tracker.row(i) = (transform*Eigen::Vector3d(tracker.row(i).transpose())).transpose();
        }
    }

    //Print(clouds,true, "clouds2");

    testOnePivotCalibration(clouds, Eigen::Vector3d(2.5,0,0),"translated x by 2.5");


    ////////////////////////////////////////////////////
    // initialize clouds again for a different rotation and translation transform
    clouds = makeCustomClouds();

    transform.setIdentity();
    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << 2.5, 0.0, 0.0;

    theta = boost::math::constants::pi<double>()/4;
    // The same rotation matrix as before; tetha radians arround Z axis
    transform.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

    for (auto&& tracker : clouds) {
        for(int i = 0; i < tracker.rows(); i++){
            // translate all of the points using an Homogenous transform matrix
            tracker.row(i) = (transform*Eigen::Vector3d(tracker.row(i).transpose())).transpose();
        }
    }

    //Print(clouds,true, "clouds3");

    testOnePivotCalibration(clouds, transform*Eigen::Vector3d(0,0,0),"translated x by 2.5, rotated by pi/4");
}

BOOST_AUTO_TEST_CASE(pivotCalibrationTest)
{
    testOnePivotCalibration(relativeDataPath, pa1debuga);
    testOnePivotCalibration(relativeDataPath, pa1debugb);
    testOnePivotCalibration(relativeDataPath, pa1debugc);
    testOnePivotCalibration(relativeDataPath, pa1debugd);
    testOnePivotCalibration(relativeDataPath, pa1debuge);
    testOnePivotCalibration(relativeDataPath, pa1debugf);
    testOnePivotCalibration(relativeDataPath, pa1debugg);
    //Print(trackerIndexedData,true,"trackerIndexedData:");
}

BOOST_AUTO_TEST_CASE(pivot2CalibrationTest)
{
    testTwoPivotCalibration(relativeDataPath, pa1debuga, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debugb, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debugc, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debugd, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debuge, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debugf, debug);
    testTwoPivotCalibration(relativeDataPath, pa1debugg, debug);
    //Print(trackerIndexedData,true,"trackerIndexedData:");
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
#endif
