#ifndef _POINT_ESTIMATION_HPP_
#define _POINT_ESTIMATION_HPP_ 


#include "matrixOperations.hpp"

/// @brief where the electromagnetic tracker estimates the electromagnetic markers are on the object
template<typename T>
std::vector<Eigen::MatrixXd> estimateCExpected(const T& calreadings, const T& calbody, bool debug = false ){
	

    std::vector<Eigen::Matrix4d> transformsEMcoordtoTrackerLocation;
    std::vector<Eigen::Matrix4d> transformsOptcoordtoTrackerLocation;
 //visitSecondTrackerRepeatedly(calreadings,calbody,
    for (int i = 0; i< calreadings.size(); ++i){
        for(int j = 0; j < 2; ++j){

            transformsEMcoordtoTrackerLocation.push_back(homogeneousInverse(hornRegistration(calreadings[i][j],calbody[0][j])));
            transformsOptcoordtoTrackerLocation.push_back(hornRegistration(calreadings[i][j],calbody[0][j]));
            if(debug) std::cout << "\n\nframes\n\n" << calbody[0][j];
        }
    }

    if(debug) std::cout << "\n\nsolveForCExpected\n\n" << transformsEMcoordtoTrackerLocation.size() << "\n\n";
	
    for (int i = 0; i<10; i++)
        std::cout << transformsEMcoordtoTrackerLocation[i] << "\n\n";
    
    std::vector<Eigen::MatrixXd> cExpected;
    for (int i = 0; i< calreadings.size(); ++i){
        for (int j = 0; j < calreadings[0][2].rows(); ++j){
            
            Eigen::Affine3d transformEM;
            transformEM.setIdentity();
            transformEM.matrix() = transformsEMcoordtoTrackerLocation[i].block<4,4>(0,0);
            
            Eigen::Affine3d transformOpt;
            transformEM.setIdentity();
            transformEM.matrix() = transformsOptcoordtoTrackerLocation[i].block<4,4>(0,0);
            
            cExpected.push_back((transformEM*transformOpt*Eigen::Vector3d(calreadings[0][2].row(j).transpose())).transpose());
        }
    }
	
	return cExpected;
}

#endif // _POINT_ESTIMATION_HPP_