#ifndef _PA2_HPP_
#define _PA2_HPP_


/// Problem 4
/// Takes positions of EM tracker points on the EM probe in the EM frame when the tip is in a CT fiducial
/// Returns points of the CT fiducial locations in EM frame
/// Need to pass dc too when making it into a function
template<typename T, typename U>
std::vector<Eigen::Vector3d> fiducialPointInEMFrame(U& em_fiducialsGVector, T& calreadingsFrames, U& cExpected,  Eigen::Vector3d centerToTip){
    std::vector<Eigen::Vector3d> fiducialPointinEMFramesReturn;
    
    
    const std::size_t homogeneousSize = 4;
    std::size_t numRowsPerTracker = em_fiducialsGVector[0].rows();
    Eigen::MatrixXd Gundistorted = correctDistortionOnSourceData(calreadingsFrames,cExpected,em_fiducialsGVector);
    std::vector<Eigen::MatrixXd> splitUndistortedFrames = splitRows(Gundistorted,numRowsPerTracker);
    Eigen::MatrixXd FtransformVector = registrationToFirstCloud(splitUndistortedFrames);
    std::vector<Eigen::MatrixXd> splitHomogeneousTransforms = splitRows(FtransformVector,homogeneousSize);
    for (auto mat:splitHomogeneousTransforms){
        Eigen::Affine3d affineFrameForEachFiducial;
        affineFrameForEachFiducial.matrix() = homogeneousInverse(mat);
        Eigen::Vector3d fiducialPointinEMFrame = affineFrameForEachFiducial*centerToTip;
        fiducialPointinEMFramesReturn.push_back(fiducialPointinEMFrame);
        std::cout << "\n\nfidicualPointinEMFrame is: \n" << fiducialPointinEMFrame << std::endl;
    }
    
    return fiducialPointinEMFramesReturn;
}


#endif // _PA2_HPP_
