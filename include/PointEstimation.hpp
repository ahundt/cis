#ifndef _POINT_ESTIMATION_HPP_
#define _POINT_ESTIMATION_HPP_ 


#include "matrixOperations.hpp"

/// @brief This function enables the electromagnetic tracker to estimate the position of the electromagnetic markers are on the object
template<typename T>
std::vector<Eigen::MatrixXd> estimateCExpected(const T& calreadings, const T& calbody, bool debug = false ){
    
    BOOST_VERIFY(!calreadings.empty() && !calbody.empty());

    std::vector<Eigen::Matrix4d> transformsEMcoordtoTrackerLocation;
    std::vector<Eigen::Matrix4d> transformsOptcoordtoTrackerLocation;
    // points with dimensions 3xnumTrackers calReadingsActualEMTrackerPositionOnCalibrationObject
    std::vector<Eigen::MatrixXd> calObjEMTrackerPosV;
    
    // load up relevant transforms
    for (int i = 0; i< calreadings.size(); ++i){
        BOOST_VERIFY(calreadings[i].size()==3);
        // only copy first two falues in here
        // perform hornRegistration and add to back of vector
        transformsEMcoordtoTrackerLocation.push_back(homogeneousInverse(hornRegistration(calreadings[i][0],calbody[0][0])));
        transformsOptcoordtoTrackerLocation.push_back(hornRegistration(calreadings[i][1],calbody[0][1]));
        calObjEMTrackerPosV.push_back(calreadings[i][2]);
    }

    if(debug) std::cout << "\n\nsolveForCExpected\n\n" << transformsEMcoordtoTrackerLocation.size() << "\n\n";
	
    for (int i = 0; i<10; i++)
        std::cout << transformsEMcoordtoTrackerLocation[i] << "\n\n";
    
    std::vector<Eigen::MatrixXd> cExpectedV;
    for (int i = 0; i< calreadings.size(); ++i){
        Eigen::Affine3d transformEM;
        transformEM.setIdentity();
        transformEM.matrix() = transformsEMcoordtoTrackerLocation[i].block<4,4>(0,0);
        
        Eigen::Affine3d transformOpt;
        transformOpt.setIdentity();
        transformOpt.matrix() = transformsOptcoordtoTrackerLocation[i].block<4,4>(0,0);
        
        Eigen::MatrixXd calObjEMTrackerPos(calObjEMTrackerPosV[i].rows(),3);
        for(std::size_t j = 0; j < calObjEMTrackerPosV[i].rows(); ++j){
            //  extract calibration Object EM Tracker Positions On Calibration Object into individual points
            Eigen::Vector3d calObjEMTrackerPosInCalibObjFrame(calObjEMTrackerPosV[i].row(j).transpose());
            Eigen::Vector3d calObjEMTrackerPosInEMFrame = transformEM*transformOpt*calObjEMTrackerPosInCalibObjFrame;
            // would be .row(j) but it is MatrixXd so we need to use .block to specify the size
            // note that the output of the transform multiplication is already in row format
            // so it can be inserted directly
            calObjEMTrackerPos.block<1,3>(j,0) = calObjEMTrackerPosInEMFrame.transpose();
        }
        cExpectedV.push_back(calObjEMTrackerPos);
    }
	
	return cExpectedV;
}

#endif // _POINT_ESTIMATION_HPP_