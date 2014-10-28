#ifndef _PIVOT_CALIBRATION_HPP_
#define _PIVOT_CALIBRATION_HPP_

#include <cstddef>
#include <iterator>
#include "hornRegistration.hpp"

/// @brief Register an STL range of tracker clouds to the first cloud in the set. Runs hornRegistration
///        on each point cloud against the other point clouds within the set.
///
/// @param tcr C++ Range of n point clouds containing tx3 Eigen::MatrixXd point clouds.
///
/// The first 4x4 matrix component returned will be I, because it is comparing against itself,
/// the subsequent matrices will be transforms from their coordinate system to the first matrix
/// coordinate system.
///
/// @return Eigen::MatrixXd Set of transforms in a combined (4*n)x4 matrix.
///
/// @todo make registration function a template param, rather than requiring hornRegistration
template<typename TrackerCloudRange>
Eigen::MatrixXd registrationToFirstCloud(const TrackerCloudRange& tcr,bool debug = false){
    static const std::size_t HtransformSize = 4;
	Eigen::MatrixXd output(std::distance(std::begin(tcr),std::end(tcr))*HtransformSize,HtransformSize);
	auto trackerCoordSysIt = std::begin(tcr);
	/// @todo eliminate finding registration on itself for first loop
    std::size_t i = 0;
	for(auto tcIt = std::begin(tcr); tcIt!=std::end(tcr); ++tcIt, i+=HtransformSize){
		output.block<HtransformSize,HtransformSize>(i,0) = hornRegistration(*tcIt,*trackerCoordSysIt);
    }
    
    if(debug) std::cout << "\n\ntransforms - registrationToFirstCloudOutput:\n\n" << output << "\n\n";
    
    return output;
}

/// @brief Split stacked transforms into stacked [Rotation,-I] matrices and stacked Position Matrices P so this is a
///
/// @return std::pair<Eigen::MatrixXd,Eigen::VectorXd> containing 3*numTransforms x 6 [Rotation,-I] matrix and 3*numTransforms x 1 position offset translation matrix p
std::pair<Eigen::MatrixXd,Eigen::VectorXd> transformToRandMinusIandPMatrices(const Eigen::MatrixXd& transforms,bool debug = false) {
    static const std::size_t HtransformSize = 4;
    static const std::size_t RotSize = 3;
    
	std::size_t transformRows = transforms.rows();
	std::size_t transformCount = transformRows/4;
	std::pair<Eigen::MatrixXd,Eigen::VectorXd> output;
	output.first.resize(transformCount*RotSize,6);
	output.second.resize(transformCount*RotSize);
    
	
	for(std::size_t transformRow = 0, outputRow = 0; transformRow < transformRows; transformRow+=HtransformSize,outputRow+=RotSize){
        
	    output.first.block<RotSize,RotSize>(outputRow,0) = transforms.block<RotSize,RotSize>(transformRow,0);
	    output.first.block<RotSize,RotSize>(outputRow,RotSize) = -Eigen::Matrix3d::Identity();
		output.second.block<RotSize,1>(outputRow,0) = transforms.block<RotSize,1>(transformRow,RotSize);
	}
    
    if(debug){
        std::cout << "\n\nRI - transformToRandMinusIandPMatrices:\n\n" << output.first << "\n\np - transformToRandMinusIandPMatrices:\n\n" << output.second << "\n\n";
    }
	
    return output;
}

/// Run SVD on the stack of matrices containing [R,-I] against the positions p
/// and solve for the common endpoint of the tracker device.
/// @pre must be at least 3 input RIp values (really [R,-I],p)
///
/// @return Eigen::VectorXd of size 6, containing ???????? @todo
Eigen::VectorXd SVDSolve(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& RIp, bool debug = false)
{
    BOOST_VERIFY(RIp.first.rows()>9);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(RIp.first, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd X = svd.solve(-RIp.second);
    if(true) std::cout << "\n\nSVDSolve - p:\n\n" << X << "\n\n";
    return X;
}

/// perform pivotCalibration, utilizing a series of tracker frames rotated around a single pivot
/// point to determine the distance from the tracker frames to the pivot point.
///
/// @param tcr A C++ Range of tracker clouds defined as Eigen::MatrixXd, with 3xn points per tracker.
/// @pre there must be at least three frames to process
/// @todo make registration function a template (and regular) parameter that passes through to registrationToFirstCloud
///
/// @return Eigen::VectorXd of size 6, the first 3 scalars are the probe tip location, the last 3 scalars are the pivot point of the probe. Both are relative to the em tracker frame.
template<typename TrackerCloudRange>
Eigen::VectorXd pivotCalibration(const TrackerCloudRange& tcr,bool debug = false){
    BOOST_VERIFY(std::distance(std::begin(tcr),std::end(tcr))>2);
    Eigen::MatrixXd transforms = registrationToFirstCloud(tcr,debug);
    //if(debug) std::cout << "\n\ntransforms - pivotCalibration:\n\n" << transforms << "\n\n";

    std::pair<Eigen::MatrixXd,Eigen::VectorXd> RIp = transformToRandMinusIandPMatrices(transforms,debug);
    //if(debug) std::cout << "\n\nRI - pivotCalibration:\n\n" << RIp.first << "\n\np - pivotCalibration:\n\n" << RIp.second << "\n\n";
    return SVDSolve(RIp,debug);
}

#endif // _PIVOT_CALIBRATION_HPP_
