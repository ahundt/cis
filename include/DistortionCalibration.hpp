#ifndef _DISTORTION_CALIBRATION_HPP_
#define _DISTORTION_CALIBRATION_HPP_

#include <boost/math/special_functions/binomial.hpp>
#include "matrixOperations.hpp"
#include "parseCSV_CIS_pointCloud.hpp"


void boundingBox(Eigen::MatrixXd& X, Eigen::Vector3d& maxCorner, Eigen::Vector3d& minCorner){
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
void ScaleToUnitBox(Eigen::MatrixXd& X, const Eigen::Vector3d& maxCorner, const Eigen::Vector3d& minCorner )
{
    // bounding box max and min
    Eigen::Vector3d diff = maxCorner-minCorner;
    for (int i=0; i<X.cols(); i++){
        for (int j=0; j<X.rows(); j++){
            // scale the x,y,z of the point
            X(j,i) = (X(j,i)-minCorner(i))/diff(i);
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
/// @param cEM n x numPoints matrix containing the c expected value, aka actual points measured by EM tracker in EM coordinate system, after translation from EM coord system
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

///
/// Solving for SVD F*C=P, where F is the EMPointsInEMFrameOnCalObj with BernsteinPolynomials applied.
///
/// @return distortion Calibration Matrix C
/// @see slide 43 of InterpolationReview.pdf
Eigen::MatrixXd distortionCalibrationMatrixC(const Eigen::MatrixXd& EMPointsInEMFrameOnCalObj, const Eigen::MatrixXd& OptPointsInEMFrameOnCalibObject ){
    
    Eigen::MatrixXd normalEMPointsInEMFrameOnCalObj(EMPointsInEMFrameOnCalObj); // aka normal cEM
    Eigen::Vector3d minCorner;
    Eigen::Vector3d maxCorner;
    boundingBox(normalEMPointsInEMFrameOnCalObj,minCorner,maxCorner);
    ScaleToUnitBox(normalEMPointsInEMFrameOnCalObj,minCorner,maxCorner); // normalize into unit box
    
    Eigen::MatrixXd FMatofEMPointsInEMFrameOnCalObj = FMatrix(normalEMPointsInEMFrameOnCalObj);
    
    std::cout << "\n\nFMatrix for SVD is rows: "<< FMatofEMPointsInEMFrameOnCalObj.rows() << " cols: " << FMatofEMPointsInEMFrameOnCalObj.cols() << std::endl << std::endl;
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(FMatofEMPointsInEMFrameOnCalObj, Eigen::ComputeThinU | Eigen::ComputeThinV);
    
    /// this is cx cy cz on slide 43 of InterpolationReview.pdf
    Eigen::MatrixXd pointCorrectionMatrix = svd.solve(OptPointsInEMFrameOnCalibObject);
    std::cout << "\n\npointCorrectionMatrix rows: " << pointCorrectionMatrix.rows() << " cols: " << pointCorrectionMatrix.cols() << "\n\n";
    
    return pointCorrectionMatrix;
}


void correctDistortion(){
    
}


/// @todo move elsewhere and remove dependency on parsing data structure
void correctDistortionOnSourceData(
                                   const csvCIS_pointCloudData::TrackerFrames& calreadingsFrames,
                                   const std::vector<Eigen::MatrixXd>&         cExpected,
                                   const csvCIS_pointCloudData::TrackerDevices& EMPtsInEMFrameOnProbe
                                   ){
    
    static const int firstFrame = 0;
    static const int IndexOptPtsInOptFrameOnEMTracker = 0;
    static const int IndexOptInOptFrameOnCalObj = 1;
    static const int IndexEMPointsInEMFrameOnCalObj = 2;
    
    BOOST_VERIFY(calreadingsFrames.size()==cExpected.size());
    BOOST_VERIFY(cExpected[0].cols()>0);
    
    int cExpectedFrameRows = cExpected[0].rows();
    int cExpectedCols = cExpected[0].cols();
    Eigen::MatrixXd cExpectedStacked(cExpected.size()*cExpectedFrameRows,cExpectedCols);
    
    static const std::size_t NumEMPointsInEMFrameOnCalObj = calreadingsFrames[firstFrame][IndexEMPointsInEMFrameOnCalObj].rows();
    static const std::size_t NumFrames = calreadingsFrames.size();
    Eigen::MatrixXd cEM;
    cEM.resize(NumEMPointsInEMFrameOnCalObj*NumFrames,3);
    
    
    for (std::size_t outputRow = 0, i = 0; i < NumFrames; outputRow+=NumEMPointsInEMFrameOnCalObj, i++){
        const Eigen::MatrixXd& markerTrackersOnCalBodyInEMFrame=calreadingsFrames[i][IndexEMPointsInEMFrameOnCalObj];
        
        // @todo For some reason putting numMarkers in for 27 does not work
        cEM.block(outputRow,0,NumEMPointsInEMFrameOnCalObj,3) = markerTrackersOnCalBodyInEMFrame;
        cExpectedStacked.block(i*cExpectedFrameRows, 0, cExpectedFrameRows, cExpectedCols);
    }
    
    Eigen::MatrixXd dcmC = distortionCalibrationMatrixC(cEM, cExpectedStacked);
    
    
    //EMPtsInEMFrameOnProbe
    
    
    //std::cout << "\n\nC size is "<< cEM.rows() << std::endl;
    //
    //std::cout << "\n\nFMatrix for SVD is \n\n"<< FMat << std::endl;
    //std::cout << "\n\nCalreadings in first frame is " << ad.calreadings.frames[0][2];
    //std::cout << "\n\nthe size is "<< ad.calreadings.frames[0][2].rows() << std::endl;
}

#endif // _DISTORTION_CALIBRATION_HPP_
