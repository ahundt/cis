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
    Eigen::VectorXd Y = 2*X;
    Eigen::MatrixXd Y3distorted = Eigen::MatrixXd::Zero(X.size(),3);
    Eigen::MatrixXd X3GroundTruth = Eigen::MatrixXd::Zero(X.size(),3);
    Y3distorted.col(0) = Y;
    X3GroundTruth.col(0) = X;
    Eigen::Vector3d minCorner;
    Eigen::Vector3d maxCorner;
    
    Eigen::MatrixXd undistorted = correctDistortion(Y3distorted, Y3distorted, X3GroundTruth, minCorner, maxCorner);
    std::cout << "\n\nUnitTestUndistorted\n\n" << undistorted << "\n\nGroundTruth\n\n" << X3GroundTruth;
    BOOST_CHECK(isWithinTolerance(undistorted,X3GroundTruth));

}
    
template<typename T>
void CompareFrames(std::string file, const T& f1, const T& f2){
    double frameTolerance = 2.0;
    auto f1b = f1.begin();
    auto f2b = f2.begin();
    for(;f1b!=f1.end(); ++f1b, ++f2b){
        auto f11b = f1b->begin();
        auto f22b = f2b->begin();
        for(;f11b != f1b->end(); ++f11b, ++f22b){
            bool isWithinToleranceB = isWithinTolerance(*f11b, *f22b, frameTolerance);
            BOOST_CHECK(isWithinToleranceB);
            if(!isWithinToleranceB){
                std::cout << "\n\n" << file << " outside of tolerances! check these values:\n\n" << *f11b << "\n\n";
                
            }
        }
        
    }
    
}
    
void CompareOutputFiles(std::string filenamePrefix){
    
    DataSource pclp;
    DataSource pclpOut;
    const bool required = true;
    // optional == not required == false
    const bool optional = false;
    // check if the user supplied a full path, if not assemble a path
    // from the default paths and the defualt prefix/suffix combos
    assemblePathIfFullPathNotSupplied(relativeOutputDataPath,filenamePrefix,dataFileNameSuffix_output1       ,pclpOut.output1Path      ,required);
    assemblePathIfFullPathNotSupplied(relativeOutputDataPath,filenamePrefix,dataFileNameSuffix_output2       ,pclpOut.output2Path      ,required);
    assemblePathIfFullPathNotSupplied(relativeDataPath,filenamePrefix,dataFileNameSuffix_output1       ,pclp.output1Path      ,required);
    assemblePathIfFullPathNotSupplied(relativeDataPath,filenamePrefix,dataFileNameSuffix_output2       ,pclp.output2Path      ,required);
    
    
    AlgorithmData ad;
    loadPointCloudFromFile(pclp.output1Path       ,ad.output1                    ,debug);
    loadPointCloudFromFile(pclp.output2Path       ,ad.output2                    ,debug);
    
    AlgorithmData adOut;
    loadPointCloudFromFile(pclpOut.output1Path       ,adOut.output1                    ,debug);
    loadPointCloudFromFile(pclpOut.output2Path       ,adOut.output2                    ,debug);
    
    //AlgorithmData adOut = initAlgorithmData(relativeOutputDataPath,filenamePrefix);
    //AlgorithmData ad = initAlgorithmData(relativeDataPath, filenamePrefix);
    
    CompareFrames(relativeOutputDataPath + filenamePrefix + dataFileNameSuffix_output1, adOut.output1.frames, ad.output1.frames);
    CompareFrames(relativeOutputDataPath + filenamePrefix + dataFileNameSuffix_output2, adOut.output2.frames, ad.output2.frames);
}
 
/// unit test output of all debug files
BOOST_AUTO_TEST_CASE(EMPivotCalibrationResults)
{
    CompareOutputFiles(pa2debuga);
    CompareOutputFiles(pa2debugb);
    CompareOutputFiles(pa2debugc);
    CompareOutputFiles(pa2debugd);
    CompareOutputFiles(pa2debuge);
    CompareOutputFiles(pa2debugf);
    std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n";
}


BOOST_AUTO_TEST_SUITE_END()
