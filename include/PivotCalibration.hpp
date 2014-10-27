#ifndef _PIVOT_CALIBRATION_HPP_
#define _PIVOT_CALIBRATION_HPP_

#include <cstddef>
#include <iterator>
#include "hornRegistration.hpp"

/// @todo make registration function a template param
template<typename TrackerCloudRange>
Eigen::MatrixXd registrationToFirstCloud(const TrackerCloudRange& tcr){
	Eigen::MatrixXd output(std::distance(std::begin(tcr),std::end(tcr))*4,4);
	auto trackerCoordSysIt = std::begin(tcr);
	/// @todo eliminate finding registration on itself for first loop
    std::size_t i = 0;
	for(auto tcIt = std::begin(tcr)+1; tcIt!=std::end(tcr); ++tcIt, ++i){
		output.block<4,4>(i*4,0) = hornRegistration(*tcIt,*trackerCoordSysIt);
	}
    return output;
}

/// @return std::pair<Eigen::MatrixXd,Eigen::VectorXd> containing 3*numTransforms x 6 [Rotation,-I] matrix and 3*numTransforms x 1 position offset translation matrix p
std::pair<Eigen::MatrixXd,Eigen::VectorXd> transformToRandMinusIandPMatrices(const Eigen::MatrixXd& transforms) {
	std::size_t transformRows = transforms.cols();
	std::size_t transformCount = transformRows/4;
	std::pair<Eigen::MatrixXd,Eigen::VectorXd> output;
	output.first.resize(transformCount*3,6);
	output.second.resize(transformCount*3);
	
	for(std::size_t transformRow = 0, outputRow = 0; transformRow < transformRows; transformRow+=4,outputRow+=3){
	    output.first.block<3,3>(transformRow,0) = transforms.block<3,3>(transformRow,0);
	    output.first.block<3,3>(transformRow,3) = -Eigen::Matrix3d::Identity();
		output.second.block<3,1>(transformRow,0) = transforms.block<3,1>(transformRow,3);
	}
	
    return output;
}

Eigen::VectorXd SVDSolve(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& RIp)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(RIp.first, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd X = svd.solve(-RIp.second);
    return X;
}

/// perform pivotCalibratioon
/// @pre there must be at least three frames to process
/// @todo figure otu what result of SVDSolve really means
/// @todo make registration function a template (and regular) parameter that passes through to registrationToFirstCloud
template<typename TrackerCloudRange>
Eigen::VectorXd pivotCalibration(const TrackerCloudRange& tcr){
    BOOST_VERIFY(std::distance(std::begin(tcr),std::end(tcr))>2);
    Eigen::MatrixXd transforms = registrationToFirstCloud(tcr);
    std::pair<Eigen::MatrixXd,Eigen::VectorXd> RIp = transformToRandMinusIandPMatrices(transforms);
    return SVDSolve(RIp);
}

#endif // _PIVOT_CALIBRATION_HPP_
