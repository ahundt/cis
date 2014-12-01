#ifndef _PA3_HPP_
#define _PA3_HPP_

#include "hornRegistration.hpp"
#include "IterativeClosestPoint.hpp"

/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh.
///
/// @pre NA and NB vectors, rather than data, must be of equal length. Corresponding to the same quantity of measured sample time points.
///
/// @param[in]  NA list of sensor data point sets for pointer A
/// @param[in]  NB list of sensor data point sets for fiducial B
/// @param[in]  Atip location of tip of pointerA in pointer A body coordinates
/// @param[in]  bodyAmarkerLEDs location of LED markers on pointer A in pointer A body coordinates
/// @param[in]  bodyAmarkerLEDs location of LED markers on fiducial B in fiducial B body coordinates
/// @param[in]  vertices list of vertices on mesh, corresponding to bone surface
/// @param[in]  vertexTriangleNeighborIndex list of 1x6 vectors. First 3 Elements are indices into vertices list, Second 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @param[out] dk location of Atip in fiducial B body coordinates, nx3 matrix of transposed vectors
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
void icpPointMeshRegistration(const std::vector<Eigen::MatrixXd>& NA,
                              const std::vector<Eigen::MatrixXd>& NB,
                              const Eigen::Vector3d& Atip,
                              const Eigen::MatrixXd& bodyAmarkerLEDs,
                              const Eigen::MatrixXd& bodyBmarkerLEDs,
                              const std::vector<Eigen::Vector3d>& vertices,
                              const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                              Eigen::MatrixXd& dk,
                              Eigen::MatrixXd& ck,
                              std::vector<double>& errork){
    
    Eigen::Vector3d ckMin;
    Eigen::Affine3d Freg;
    Freg.setIdentity();
	
	ck.resize(NA.size(),3);
	dk.resize(NA.size(),3);
    
    for (int i=0; i<NA.size(); i++){
        double errorMin=std::numeric_limits<double>::max();
        Eigen::Affine3d FaAffine(hornRegistration(bodyAmarkerLEDs,NA[i])); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
        Eigen::Affine3d FbInverseAffine(hornRegistration(NB[i], bodyBmarkerLEDs)); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB

        // Atip: Problem3-BodyA (last line)
        Eigen::Vector3d dk_i(Eigen::Vector3d(FbInverseAffine*FaAffine*Atip));
		dk.block<1,3>(i,0) = dk_i.transpose();

        Eigen::Vector3d sk(Freg*dk_i);
        for (auto&& triangle : vertexTriangleNeighborIndex){
            Eigen::Vector3d ckTemp = FindClosestPoint(sk, vertices, triangle);
            double errorTemp = (ckTemp-dk_i).norm();
            if (errorTemp < errorMin){
                ckMin = ckTemp;
                errorMin = errorTemp;
            }
        }
		ck.block<1,3>(i,0) = ckMin.transpose();
        errork.push_back(errorMin);
    }
	
	// NOTE: this step may be wrong!
	Freg = hornRegistration(ck,dk);
}

#endif // _PA3_HPP_