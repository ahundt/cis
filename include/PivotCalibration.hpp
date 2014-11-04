#ifndef _PIVOT_CALIBRATION_HPP_
#define _PIVOT_CALIBRATION_HPP_

#include <cstddef>
#include <iterator>
#include "hornRegistration.hpp"

/// @brief Register an STL range of tracker clouds to the first cloud in the set. Runs hornRegistration
///        on each point cloud against the other point clouds within the set.
///
/// @param cloudA C++ Range of n point clouds containing tx3 Eigen::MatrixXd point clouds.
///
/// The first 4x4 matrix component returned will be I, because it is comparing against itself,
/// the subsequent matrices will be transforms from their coordinate system to the first matrix
/// coordinate system.
///
/// @return Eigen::MatrixXd Set of transforms in a combined (4*n)x4 matrix.
///
/// @todo make registration function a template param, rather than requiring hornRegistration
template<typename TrackerCloudRange>
Eigen::MatrixXd registrationToFirstCloud(const TrackerCloudRange& cloudA,bool debug = false){
    static const std::size_t HtransformSize = 4;
	Eigen::MatrixXd output(std::distance(std::begin(cloudA),std::end(cloudA))*HtransformSize,HtransformSize);

    // take the first cloud
	auto trackerCoordSysCloudAIt = std::begin(cloudA); // For Problem 6, will ask Paul if we need to find the h from original H data or from FDinv*H

    // find the center aka mean point
    Eigen::Vector3d abar;
    Eigen::MatrixXd cloudAMoved = *trackerCoordSysCloudAIt;
    moveCloudToOrigin(cloudAMoved, abar);
	/// @todo eliminate finding registration on itself for first loop
    std::size_t i = 0;
	for(auto cloudIt = std::begin(cloudA); cloudIt!=std::end(cloudA); ++cloudIt, i+=HtransformSize){
		output.block<HtransformSize,HtransformSize>(i,0) = hornRegistration(cloudAMoved,*cloudIt);
    }

    if(debug) std::cout << "\n\ntransforms - registrationToFirstCloudOutput:\n\n" << output << "\n\n";

    return output;
}

// Assumes the inverse of the homogeneous matrix of cloudA2 needs to be taken
template<typename TrackerCloudRange>
Eigen::MatrixXd registrationToTwoSeriallyLinkedClouds(const TrackerCloudRange& cloudA1, const TrackerCloudRange& cloudA2, bool debug = false){
    static const std::size_t HtransformSize = 4;
	Eigen::MatrixXd output(std::distance(std::begin(cloudA1),std::end(cloudA1))*HtransformSize,HtransformSize);
    // take the first cloud
    auto trackerCoordSysCloudA1It = std::begin(cloudA1);
    auto trackerCoordSysCloudA2It = std::begin(cloudA2);
    Eigen::MatrixXd cloudA1Moved = *trackerCoordSysCloudA1It;
    Eigen::MatrixXd cloudA2Moved = *trackerCoordSysCloudA2It;
    Eigen::Vector3d a1bar;
    Eigen::Vector3d a2bar;
    moveCloudToOrigin(cloudA1Moved, a1bar);
    moveCloudToOrigin(cloudA2Moved, a2bar);

    std::size_t i = 0;
	for(auto cloudIt1 = std::begin(cloudA1), cloudIt2 = std::begin(cloudA2); cloudIt1!=std::end(cloudA1); ++cloudIt1, ++cloudIt2, i+=HtransformSize){
        Eigen::MatrixXd FcloudA2inv = hornRegistration(cloudA2Moved,*cloudIt2); //This is H, it calculates FH
        Eigen::MatrixXd FcloudA = hornRegistration(*cloudIt1,cloudA1Moved); //This is D, it calculates FDinv
        output.block<HtransformSize,HtransformSize>(i,0) = FcloudA*FcloudA2inv; //F = FDinv*FH
        if(debug){
            std::cout << "\n\nFtrc2inv:\n\n" << FcloudA2inv << "\n\nFcloudA:\n\n" << FcloudA << "\n\nFcloudA2inv*FcloudA:\n\n" << FcloudA2inv*FcloudA << "\n\n";
        }
    }

    if(debug) {
        std::cout << "\n\nregistrationToFirstCloudOutput:\n\n" << output << "\n\n";
    }

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
	output.first.resize(transformCount*RotSize-RotSize,6);
	output.second.resize(transformCount*RotSize-RotSize);


	for(std::size_t transformRow = HtransformSize, outputRow = 0; transformRow < transformRows; transformRow+=HtransformSize,outputRow+=RotSize){

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
Eigen::VectorXd SVDSolveRIp(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& RIp, bool debug = false)
{
    BOOST_VERIFY(RIp.first.rows()>9);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(RIp.first, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd X = svd.solve(-RIp.second);
    if(debug) std::cout << "\n\nSVDSolveRIp - p:\n\n" << X << "\n\n";
    return X;
}

/// perform pivotCalibration, utilizing a series of tracker frames rotated around a single pivot
/// point to determine the distance from the tracker frames to the pivot point.
///
/// @param cloudA A C++ Range of tracker clouds defined as Eigen::MatrixXd, with 3xn points per tracker. Typically vector<MatrixXd>
/// @pre there must be at least three frames to process
/// @todo make registration function a template (and regular) parameter that passes through to registrationToFirstCloud
///
/// @return Eigen::VectorXd of size 6, the first 3 scalars are the probe tip location, the last 3 scalars are the pivot point of the probe. Both are relative to the em tracker frame.
template<typename TrackerCloudRange>
Eigen::VectorXd pivotCalibration(const TrackerCloudRange& cloudA,bool debug = false){
    BOOST_VERIFY(std::distance(std::begin(cloudA),std::end(cloudA))>2);
    Eigen::MatrixXd transforms = registrationToFirstCloud(cloudA,debug);
    //if(debug) std::cout << "\n\ntransforms - pivotCalibration:\n\n" << transforms << "\n\n";

    std::pair<Eigen::MatrixXd,Eigen::VectorXd> RIp = transformToRandMinusIandPMatrices(transforms,debug);
    //if(debug) std::cout << "\n\nRI - pivotCalibration:\n\n" << RIp.first << "\n\np - pivotCalibration:\n\n" << RIp.second << "\n\n";

    Eigen::VectorXd X =  SVDSolveRIp(RIp,debug);
    if(debug) std::cout << "\n\npivotCalibration - p:\n\n" << X << "\n\n";
    return X;
}

/// perform pivotCalibration of Optical Probe in terms of the EM Coordinate System
/// @pre there must be at least three frames to process
///
/// @param cloudA A C++ Range of tracker clouds defined as Eigen::MatrixXd, with 3xn points per tracker.
/// @param cloudA2 A C++ Range of tracker clouds defined as Eigen::MatrixXd, with 3xn points per tracker.
/// @pre there must be at least three frames to process
/// @todo make registration function a template (and regular) parameter that passes through to registrationToFirstCloud
///
/// @return Eigen::VectorXd of size 6, the first 3 scalars are the probe tip location, the last 3 scalars are the pivot point of the probe. Both are relative to the em tracker frame.
template<typename TrackerCloudRange>
Eigen::VectorXd pivotCalibrationTwoSystems(const TrackerCloudRange& cloudA, const TrackerCloudRange& cloudA2,bool debug = false){
    BOOST_VERIFY(std::distance(std::begin(cloudA),std::end(cloudA))>2);
    BOOST_VERIFY(std::distance(std::begin(cloudA2),std::end(cloudA2))>2);
    //Eigen::MatrixXd transforms = registrationToFirstCloud(cloudA,debug);
    Eigen::MatrixXd transforms = registrationToTwoSeriallyLinkedClouds(cloudA, cloudA2, debug);
    std::pair<Eigen::MatrixXd,Eigen::VectorXd> RIp = transformToRandMinusIandPMatrices(transforms, debug);
    //if(debug) std::cout << "\n\nRI:\n\n" << RIp.first << "\n\np:\n\n" << RIp.second << "\n\n";
    return SVDSolveRIp(RIp,debug);
}



#endif // _PIVOT_CALIBRATION_HPP_
