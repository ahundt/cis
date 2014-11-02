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

/// @brief This function enables a probe tracked in the Optical frame to be put in EM coordinates
/// @todo Document what is G? Explain in english.
template<typename T>
std::vector<Eigen::MatrixXd> findNewGInEM(const T& optpivot, const T& calbody, bool debug = false ){
    // Sorts through the input files and splits it into vectors of vectors
    // trackerIndexedData[0][i] is D and [1][i] is G
    csvCIS_pointCloudData::TrackerFrames trackerIndexedData(swapIndexing(optpivot));

    BOOST_VERIFY(!optpivot.empty() && !calbody.empty());
    // opticalMarkersOnEMDeviceInEMFrame = d from calbody
    Eigen::MatrixXd opticalMarkersOnEMDeviceInEMFrame(calbody[0][0]); // A

    // Intializes a vector of Eigen Matrices to store Gnew
    std::vector<Eigen::MatrixXd> Gnew;

    for (int i = 0; i< trackerIndexedData[0].size(); ++i){
        // Confirms that there are two data sets in trackerIndexedData (D and G)
        BOOST_VERIFY(trackerIndexedData.size()==2);
        // only copy first two values in here
        // perform hornRegistration and add to back of vector

        // input points are in EM Tracker frame
        // output points are in Optical Tracker Frame
        // Find transform from the probe optical tracker reference frame to the EM coordinates of the Probe

        // trackerIndexedData[1][i] = G for each frame from optpivot
        Eigen::MatrixXd OptMarkersOnProbeInOptFrame(trackerIndexedData[1][i]);

        // trackerIndexedData[0][i] = D for each frame from optpivot
        Eigen::MatrixXd OptMarkersOnEMDeviceInOptFrame(trackerIndexedData[0][i]); // D

        // Creates the Horn Transform from d to D (i.e. FDinv) in each frame
        Eigen::Affine3d OptToEMFrame(hornRegistration( // horn(D,A)
                                                        OptMarkersOnEMDeviceInOptFrame,
                                                        opticalMarkersOnEMDeviceInEMFrame
                                                        ));

        Eigen::MatrixXd probePtsInEMTrackerFrame(OptMarkersOnProbeInOptFrame.rows(),3);
        for(std::size_t j = 0; j < OptMarkersOnProbeInOptFrame.rows(); ++j){
            //  extract Optical Tracker Positions On Probe into individual points
            Eigen::Vector3d OptMarkerPointsOnProbeInOptFrame(OptMarkersOnProbeInOptFrame.row(j).transpose()); // Extract point from C

            // starting with point on calibration object in calObj frame -> optFrame -> EMFrame
            Eigen::Vector3d probeOptTrackerPosInEMFrame = OptToEMFrame*OptMarkerPointsOnProbeInOptFrame;

            // would be .row(j) but it is MatrixXd so we need to use .block to specify the size
            // note that the output of the transform multiplication is already in row format
            // so it can be inserted directly

            // Stores the Gnew value of the current frame (Gnew = FDinv*G)
            probePtsInEMTrackerFrame.block<1,3>(j,0) = probeOptTrackerPosInEMFrame.transpose();
        }
        // Puts the Eigen Matrix Gnew into a vector corresponding to the frame
        Gnew.push_back(probePtsInEMTrackerFrame);
    }
    return Gnew;
}

#endif // _POINT_ESTIMATION_HPP_
