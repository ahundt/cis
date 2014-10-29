#ifndef _POINT_ESTIMATION_HPP_
#define _POINT_ESTIMATION_HPP_ 


#include "matrixOperations.hpp"

/// @brief This function enables the electromagnetic tracker to estimate the position of the electromagnetic markers are on the object
template<typename T>
std::vector<Eigen::MatrixXd> estimateCExpected(const T& calreadings, const T& calbody, bool debug = false ){
    
    BOOST_VERIFY(!calreadings.empty() && !calbody.empty());

    // points with dimensions 3xnumTrackers calReadingsActualEMTrackerPositionOnCalibrationObject
    Eigen::MatrixXd calObjEMTrackerPoints;
    
    
    Eigen::MatrixXd opticalMarkersOnEMDeviceInEMFrame(calbody[0][0]); // A
    Eigen::MatrixXd OptMarkersOnCalObjInCalObjFrame(calbody[0][1]);   // B
    Eigen::MatrixXd EMMarkersOnCalObjInCalObjFrame(calbody[0][2]);    // C
    
    
    std::vector<Eigen::MatrixXd> cExpectedV; // expected output
    
    // load up relevant transforms
    for (int i = 0; i< calreadings.size(); ++i){
        BOOST_VERIFY(calreadings[i].size()==3);
        // only copy first two falues in here
        // perform hornRegistration and add to back of vector
        
        // input points are in EM Tracker frame
        // output points are in Optical Tracker Frame
        // Find transform from the calibration object reference frame to the EM coordinates of the calibration object
        Eigen::MatrixXd OptMarkersOnEMDeviceInOptFrame(calreadings[i][0]); // D
        Eigen::Affine3d OptToEMFrame(hornRegistration( // horn(D,A)
                                                      OptMarkersOnEMDeviceInOptFrame,
                                                      opticalMarkersOnEMDeviceInEMFrame
                                                      ));
        
        // input points are in calibration object frame
        // output points are in optical tracker frame
        // Find transform from the calibration object reference frame to the EM coordinates of the calibration object
        Eigen::MatrixXd OptMarkersOnCalObjectInOptFrame(calreadings[i][1]); // E
        Eigen::Affine3d CalObjToOptFrame(hornRegistration( // horn(B,E)
                                                          OptMarkersOnCalObjInCalObjFrame,
                                                          OptMarkersOnCalObjectInOptFrame
                                                          ));
        
        
        Eigen::MatrixXd calObjPtsInEMTrackerF(EMMarkersOnCalObjInCalObjFrame.rows(),3);
        for(std::size_t j = 0; j < EMMarkersOnCalObjInCalObjFrame.rows(); ++j){
            //  extract calibration Object EM Tracker Positions On Calibration Object into individual points
            Eigen::Vector3d EMMarkerOnCalObjInCalObjFrame(EMMarkersOnCalObjInCalObjFrame.row(j).transpose()); // Extract point from C
            
            // starting with point on calibration object in calObj frame -> optFrame -> EMFrame
            Eigen::Vector3d calObjEMTrackerPosInEMFrame = OptToEMFrame*CalObjToOptFrame*EMMarkerOnCalObjInCalObjFrame;
            
            // would be .row(j) but it is MatrixXd so we need to use .block to specify the size
            // note that the output of the transform multiplication is already in row format
            // so it can be inserted directly
            calObjPtsInEMTrackerF.block<1,3>(j,0) = calObjEMTrackerPosInEMFrame.transpose();
        }
        
        cExpectedV.push_back(calObjPtsInEMTrackerF);
    }

	
	return cExpectedV;
}

#endif // _POINT_ESTIMATION_HPP_