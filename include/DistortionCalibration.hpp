#ifndef _DISTORTION_CALIBRATION_HPP_
#define _DISTORTION_CALIBRATION_HPP_

#include <limits>
#include <boost/math/special_functions/binomial.hpp>
#include "matrixOperations.hpp"
#include "parseCSV_CIS_pointCloud.hpp"


void boundingBox(Eigen::MatrixXd& X, Eigen::Vector3d& minCorner, Eigen::Vector3d& maxCorner){
    maxCorner = X.colwise().maxCoeff();
    minCorner = X.colwise().minCoeff();
}

/// Scales every dimension of the point cloud to the max and min values
/// Also finds the original bounding box which is subsequently scaled down.
///
/// @todo what are the expected dimensions of X?
/// @todo which return row is the min and which is the max
/// @todo this does not scale to the unit box, it scales to the size of the max and min point of the matrix.
///
/// @param X nx3 matrix containing points that will be scaled
/// @param maxCorner the maximum coordinate in all dimensions of the bounding box
/// @param ignoreBounds ignore if coordinates are not between 0 and 1. Defaults to false, which means there is an assertion checking the bounds.
void ScaleToUnitBox(Eigen::MatrixXd& X, const Eigen::Vector3d& minCorner, const Eigen::Vector3d& maxCorner, bool ignoreBounds = false )
{
    // bounding box max and min
    Eigen::Vector3d diff = maxCorner-minCorner;
    /// @todo come up with better way to handle when diff is 0
    if(diff(0)==0) diff(0) = 1;
    if(diff(1)==0) diff(1) = 1;
    if(diff(2)==0) diff(2) = 1;
    
    for (int i=0; i<X.cols(); i++){
        for (int j=0; j<X.rows(); j++){
            // scale the x,y,z of the point
            auto coord = (diff(i)==0.0) ? 0.0 : (X(j,i)-minCorner(i))/diff(i);
            if(!ignoreBounds){
                BOOST_VERIFY(coord <= 1); // verify scaling is working
                BOOST_VERIFY(coord >= 0);
            }
            X(j,i) = coord;
        }
    }
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
/// @param N the polynomial degree
/// @see slide 42 and 43 of InterpolationReview.pdf
/// @todo advanced implementation: template on the polynomial size
Eigen::MatrixXd FMatrixRow(const Eigen::Vector3d& v,int N = 5, bool debug = false)
{
    int index = 0; // position in the output matrix
    int columns = pow(N+1,3);
    Eigen::MatrixXd F(1,columns);
    //std::cout << "\n\nF is " << F << std::endl;
    for (int i=0; i<=N; i++){
        for(int j=0; j<=N; j++){
            for(int k=0; k<=N; k++){
                BOOST_VERIFY(index<columns);
                // Fijk = Bi * Bj * Bk F(0,index)
                //std::cout << "\n\nindex is " << index << std::endl;
                double bSum = BersteinPolynomial(v(0),N,i)*BersteinPolynomial(v(1),N,j)*BersteinPolynomial(v(2),N,k);
                //std::cout << "\n\nB is " << B << std::endl;
                F.block<1,1>(0,index) << bSum;
                index++;
                //std::cout << "\n\nF is " << F << std::endl;
            }
        }
    }
    
    if(debug){
        std::cout << "\n\ncolumns is " << columns << std::endl;
        std::cout << "\n\nFMatrixRow:\n\n" << F << "\n\n";
    }
    return F;
}


/// Normalize the cEM matrix of points, then find the
/// F Matrix row of each point and insert it into a larger
/// matrix on which SVD will be solved.
///
/// @pre cEM must be normalized to the unit rectangle
///
/// @see slide 42 and 43 of InterpolationReview.pdf
///
///
/// @param cEM numPoints x n (with n=3 normally) matrix containing the c expected value, aka actual points measured by EM tracker in EM coordinate system, after translation from EM coord system
/// @param N the polynomial degree
Eigen::MatrixXd FMatrix(const Eigen::MatrixXd& normalcEM, int N = 5, bool debug = false){
    /// @todo don't recompute pow here and in FMatrixRow
    int columns = pow(N+1,3);
    int rows = normalcEM.rows();
    Eigen::MatrixXd cEMFMatrix(rows,columns);
    
    
    for (int i=0; i<rows; i++){
        Eigen::Vector3d vXYZ;
        vXYZ = normalcEM.block<1,3>(i,0);
        Eigen::MatrixXd row = FMatrixRow(vXYZ,N,debug);
        if(debug) std::cout << "\n\nreturned FMatrixRow:\n\n" << row << "\n\n";
        cEMFMatrix.row(i) = row;
    }
    if(debug) std::cout << "\n\ncEMFMatrix:\n\n" << cEMFMatrix << "\n\n";
    return cEMFMatrix;
}

/// Take a vector of matrices and stack it vertically into one large matrix
/// with the first matrix in the vector at the top and the last at the bottom.
///
/// @pre assumes all matrices have the same dimensions
template<typename T>
Eigen::MatrixXd stackRange(const T & vecMat){
    auto begin = std::begin(vecMat);
    auto end = std::end(vecMat);
    auto distance = std::distance(begin,end);
    if(!distance) return Eigen::MatrixXd();
    
    std::size_t rows = begin->rows();
    std::size_t cols = begin->cols();
    Eigen::MatrixXd stack(rows*distance,cols);
    
    std::size_t i = 0;
    for(auto mat : vecMat ){
        stack.block(i*rows, 0, rows, cols) = mat;
        ++i;
    }
    
    return stack;
}

/// Take a vector of Vector3d (or points) and stack the transpose of each vector (aka row vector)
/// vertically into one large matrix with the first Vector3d in the vector at the top and the last
/// at the bottom.
///
/// @note Currently only works with vectors
///
/// @pre assumes all matrices have the same dimensions
template<typename T>
Eigen::MatrixXd stackRangeTranspose(const T & vecMat){
    auto begin = std::begin(vecMat);
    auto end = std::end(vecMat);
    auto distance = std::distance(begin,end);
    if(!distance) return Eigen::MatrixXd();
    
    std::size_t rows = begin->rows();
    std::size_t cols = begin->cols();
    Eigen::MatrixXd stack(distance,rows);
    
    std::size_t i = 0;
    for(auto mat : vecMat ){
        stack.block(i, 0, cols, rows) = mat.transpose();
        ++i;
    }
    
    return stack;
}

/// Takes a set of points and converts it to a matrix of normalized points aka points scaled to the unit box,
/// where they are subsequently used to calculate F values for SVD.
///
/// @see slide 43 of InterpolationReview.pdf
///
/// @param pointInAllFrames an numPoints x 3 matrix cointaining all the points to be normalized and inserted into an F Matrix for solving with SVD
/// @param[out] minCorner the minimum coordinate of the distorted parameter, used for scaling to the unit box
/// @param[out] maxCorner the maximum coordinate of the distorted parameter, used for scaling to the unit box
Eigen::MatrixXd normalizedFMatrix(const Eigen::MatrixXd& pointsInAllFrames, Eigen::Vector3d& minCorner, Eigen::Vector3d& maxCorner)
{
    Eigen::MatrixXd pointsNormalizedToUnitBox(pointsInAllFrames); // aka normal cEM
    boundingBox(pointsNormalizedToUnitBox,minCorner,maxCorner);
    ScaleToUnitBox(pointsNormalizedToUnitBox,minCorner,maxCorner); // normalize into unit box
    
    Eigen::MatrixXd FMatForSVD = FMatrix(pointsNormalizedToUnitBox);
    
    return FMatForSVD;
}

///
/// Solving for SVD F*C=P, where F is the EMPointsInEMFrameOnCalObj with BernsteinPolynomials applied.
///
/// @return distortion Calibration Matrix C
/// @see slide 43 of InterpolationReview.pdf
Eigen::MatrixXd distortionCalibrationMatrixC(const Eigen::MatrixXd& EMPointsInEMFrameOnCalObj, const Eigen::MatrixXd& OptPointsInEMFrameOnCalibObject, Eigen::Vector3d& minCorner, Eigen::Vector3d& maxCorner ){
    
    Eigen::MatrixXd FMatofEMPointsInEMFrameOnCalObj = normalizedFMatrix(EMPointsInEMFrameOnCalObj, minCorner, maxCorner);
    std::cout << "\n\nFMatrix for SVD is rows: "<< FMatofEMPointsInEMFrameOnCalObj.rows() << " cols: " << FMatofEMPointsInEMFrameOnCalObj.cols() << std::endl << std::endl;
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(FMatofEMPointsInEMFrameOnCalObj, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    /// this is cx cy cz on slide 43 of InterpolationReview.pdf
    Eigen::MatrixXd pointCorrectionMatrix = svd.solve(OptPointsInEMFrameOnCalibObject);
    std::cout << "\n\npointCorrectionMatrix rows: " << pointCorrectionMatrix.rows() << " cols: " << pointCorrectionMatrix.cols() << "\n\n";
    
    return pointCorrectionMatrix;
}


/// Correct distortions in one point cloud by utilizing distorted and undistorted versions of a second point cloud.
/// Bernstein Polynomials are utilized to perform the correction.
///
/// @param[in] distortedToCorrect the distorted data set to correct
/// @param[in] distortedGroundTruth the same data as groundTruth, but this data has distortion, and the variation between this and the real groundTruth will be used to correct distortedToCorrect.
/// @param[in] groundTruth previously known exact values with no distortion to determine the coefficient matrix to correct the distortion
/// @param[out] minCorner the minimum coordinate of the distorted parameter, used for scaling to the unit box
/// @param[out] maxCorner the maximum coordinate of the distorted parameter, used for scaling to the unit box
///
/// @return Eigen::MatrixXd containing data that should match groundTruth
Eigen::MatrixXd correctDistortion(const Eigen::MatrixXd& distortedToCorrect, const Eigen::MatrixXd& distortedGroundTruth, const Eigen::MatrixXd& groundTruth, Eigen::Vector3d& minCorner, Eigen::Vector3d& maxCorner){
    
    Eigen::MatrixXd dcmC = distortionCalibrationMatrixC(distortedGroundTruth, groundTruth,minCorner,maxCorner);
    
    // scale using the same scaling factor as before, ignoring if it doesn't fit in the 0 to 1 bounds
    // this bool only affects a BOOST_VERIFY check, not function program behavior.
    bool ignoreUnitBoxScalingBounds = true;
    Eigen::MatrixXd distortedToCorrectScaled = distortedToCorrect;
    ScaleToUnitBox(distortedToCorrectScaled, minCorner, maxCorner,ignoreUnitBoxScalingBounds);
    
    Eigen::MatrixXd FMatrixDistorted = FMatrix(distortedToCorrectScaled);
    //               corrected distortion matrix =        F*C
    Eigen::MatrixXd undistorted = FMatrixDistorted*dcmC;
    
    return undistorted;
}


/// @todo move elsewhere and remove dependency on parsing data structure
template<typename T, typename U>
Eigen::MatrixXd correctDistortionOnSourceData(
                                   const T& calreadingsFrames, // typicaly std::vector<std::vector<Eigen::MatrixXd> >
                                   const std::vector<Eigen::MatrixXd>&         cExpected,
                                   const U& EMPtsInEMFrameOnProbe  // typicaly std::vector<std::vector<Eigen::MatrixXd> >
                                   ){
    
    static const int firstFrame = 0;
    static const int IndexOptPtsInOptFrameOnEMTracker = 0;
    static const int IndexOptInOptFrameOnCalObj = 1;
    static const int IndexEMPointsInEMFrameOnCalObj = 2;
    
    
    BOOST_VERIFY(calreadingsFrames.size()==cExpected.size());
    BOOST_VERIFY(cExpected[0].cols()>0);
    
    // create stacked version of cExpected
    Eigen::MatrixXd cExpectedStacked = stackRange(cExpected);
    
    // prep cEM for manual stacking since it is a vector of vectors
    // Stack EM Points in EM frame on to cEM matrix
    static const std::size_t NumEMPointsInEMFrameOnCalObj = calreadingsFrames[firstFrame][IndexEMPointsInEMFrameOnCalObj].rows();
    static const std::size_t NumFrames = calreadingsFrames.size();
    Eigen::MatrixXd cEM;
    cEM.resize(NumEMPointsInEMFrameOnCalObj*NumFrames,3);
    for (std::size_t outputRow = 0, i = 0; i < NumFrames; outputRow+=NumEMPointsInEMFrameOnCalObj, i++){
        const Eigen::MatrixXd& markerTrackersOnCalBodyInEMFrame=calreadingsFrames[i][IndexEMPointsInEMFrameOnCalObj];
        // @todo For some reason putting numMarkers in for 27 does not work
        cEM.block(outputRow,0,NumEMPointsInEMFrameOnCalObj,3) = markerTrackersOnCalBodyInEMFrame;
    }
    
    Eigen::Vector3d minCorner;
    Eigen::Vector3d maxCorner;
    
    auto StackedEMPtsInEMFrameOnProbe = stackRange(EMPtsInEMFrameOnProbe);
    
    Eigen::MatrixXd undistortedEMPointsInEMFrame = correctDistortion(StackedEMPtsInEMFrameOnProbe, cEM, cExpectedStacked, minCorner, maxCorner);
    
    return undistortedEMPointsInEMFrame;
}



#endif // _DISTORTION_CALIBRATION_HPP_
