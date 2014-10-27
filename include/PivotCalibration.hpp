#ifndef _PIVOT_CALIBRATION_HPP_
#define _PIVOT_CALIBRATION_HPP_

#include <cstddef>
#include <iterator>
#include "hornRegistration.hpp"

/// @todo make registration function a template param
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
    std::cout << "\n\nregistrationToFirstCloudOutput:\n\n" << output << "\n\n";
    return output;
}

/// @return std::pair<Eigen::MatrixXd,Eigen::VectorXd> containing 3*numTransforms x 6 [Rotation,-I] matrix and 3*numTransforms x 1 position offset translation matrix p
std::pair<Eigen::MatrixXd,Eigen::VectorXd> transformToRandMinusIandPMatrices(const Eigen::MatrixXd& transforms,bool debug = false) {
	std::size_t transformRows = transforms.rows();
	std::size_t transformCount = transformRows/4;
	std::pair<Eigen::MatrixXd,Eigen::VectorXd> output;
	output.first.resize(transformCount*3,6);
	output.second.resize(transformCount*3);
    
	
	for(std::size_t transformRow = 0, outputRow = 0; transformRow < transformRows; transformRow+=4,outputRow+=3){
        
	    output.first.block<3,3>(outputRow,0) = transforms.block<3,3>(transformRow,0);
	    output.first.block<3,3>(outputRow,3) = -Eigen::Matrix3d::Identity();
		output.second.block<3,1>(outputRow,0) = transforms.block<3,1>(transformRow,3);
	}
    
    if(debug){
        std::cout << "\n\ntransformToRandMinusIandPMatricesOutput\n\n";
        std::cout << "\n\nRI:\n\n" << output.first << "\n\np:\n\n" << output.second << "\n\n";
    }
	
    return output;
}

/// @pre must be at least 3 input RIp values (really [R,-I],p)
Eigen::VectorXd SVDSolve(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& RIp)
{
    BOOST_VERIFY(RIp.first.rows()>9);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(RIp.first, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd X = svd.solve(-RIp.second);
    return X;
}

/// perform pivotCalibration
/// @pre there must be at least three frames to process
/// @todo figure otu what result of SVDSolve really means
/// @todo make registration function a template (and regular) parameter that passes through to registrationToFirstCloud
template<typename TrackerCloudRange>
Eigen::VectorXd pivotCalibration(const TrackerCloudRange& tcr,bool debug = false){
    BOOST_VERIFY(std::distance(std::begin(tcr),std::end(tcr))>2);
    Eigen::MatrixXd transforms = registrationToFirstCloud(tcr,debug);
    std::pair<Eigen::MatrixXd,Eigen::VectorXd> RIp = transformToRandMinusIandPMatrices(transforms,debug);
    if(debug) std::cout << "\n\nRI:\n\n" << RIp.first << "\n\np:\n\n" << RIp.second << "\n\n";
    return SVDSolve(RIp);
}

#endif // _PIVOT_CALIBRATION_HPP_
