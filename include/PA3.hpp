#ifndef _PA3_HPP_
#define _PA3_HPP_



#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <cmath>
#include <vector>
#include <iostream>
#include <boost/foreach.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;



#include "hornRegistration.hpp"
#include "IterativeClosestPoint.hpp"

typedef Eigen::Vector3d EV3d;
typedef bg::cs::cartesian bgcscart;

#define ZEROINDEX operator()(0)
#define ONEINDEX operator()(1)
#define TWOINDEX operator()(2)

BOOST_GEOMETRY_REGISTER_POINT_3D(EV3d, double, bgcscart, ZEROINDEX, ONEINDEX, TWOINDEX)


namespace cis {
    
    //typedef bg::model::point<double, 3, bg::cs::cartesian> point;
    typedef Eigen::Vector3d point;
    typedef bg::model::box<point> box;
    typedef bg::model::polygon<point, false, false> polygon; // ccw, open polygon
    typedef std::pair<box, polygon> value;
}


/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh.
///
/// @pre NA and NB vectors, rather than data, must be of equal length. Corresponding to the same quantity of measured sample time points.
///
/// @param[in]  NA list of sensor data point sets for pointer A
/// @param[in]  NB list of sensor data point sets for fiducial B
/// @param[in]  Atip location of tip of pointerA in pointer A body coordinates
/// @param[in]  bodyAmarkerLEDs location of LED markers on pointer A in pointer A body coordinates
/// @param[in]  bodyAmarkerLEDs location of LED markers on fiducial B in fiducial B body coordinatesnd 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @return dkList location of Atip in fiducial B body coordinates, nx3 matrix of transposed vectors
Eigen::MatrixXd
dkKnownMeshPointsBaseFrame(const std::vector<Eigen::MatrixXd>& NA,
                                const std::vector<Eigen::MatrixXd>& NB,
                                const Eigen::Vector3d& Atip,
                                const Eigen::MatrixXd& bodyAmarkerLEDs,
                                const Eigen::MatrixXd& bodyBmarkerLEDs){
    
    Eigen::MatrixXd dkList(NA.size(),3);
    
    for (int i=0; i<NA.size(); i++){
        Eigen::Affine3d FaAffine(hornRegistration(bodyAmarkerLEDs,NA[i])); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
        Eigen::Affine3d FbInverseAffine(hornRegistration(NB[i], bodyBmarkerLEDs)); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
        
        // Atip: Problem3-BodyA (last line)
        Eigen::Vector3d dk_i(Eigen::Vector3d(FbInverseAffine*FaAffine*Atip));
        dkList.block<1,3>(i,0) = dk_i.transpose();
    }
    
    return dkList;
}

/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh.
///
/// @param[out] dkList location of Atip in fiducial B body coordinates, nx3 matrix of transposed vectors
/// @param[in]  vertices list of vertices on mesh, corresponding to bone surface
/// @param[in]  vertexTriangleNeighborIndex list of 1x6 vectors. First 3 Elements are indices into vertices list, Second 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
void icpPointMeshRegistration(
                              const Eigen::MatrixXd& dkList,
                              const std::vector<Eigen::Vector3d>& vertices,
                              const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                              Eigen::Affine3d& Freg,
                              Eigen::MatrixXd& skList,
                              Eigen::MatrixXd& ckList,
                              std::vector<double>& errork){
    
    errork.clear();
    ckList.resize(dkList.rows(),3);
    skList.resize(dkList.rows(),3);
    
    for (int i=0; i<dkList.rows(); i++){
        double errorMin=std::numeric_limits<double>::max();
	    Eigen::Vector3d ckMin;
        Eigen::Vector3d dk_i(dkList.block<1,3>(i,0).transpose());
        
        Eigen::Vector3d sk(Freg*dk_i);
        skList.block<1,3>(i,0) = sk.transpose();
        
        //if(i % NA.size() == 3) std::cout << "\n\nsk[3]\n\n" << sk << "\n\n";
        for (auto&& triangle : vertexTriangleNeighborIndex){
            Eigen::Vector3d ckTemp = FindClosestPoint(sk, vertices, triangle);
            double errorTemp = (sk-ckTemp).norm();
            if (errorTemp < errorMin){
                ckMin = ckTemp;
                errorMin = errorTemp;
            }
        }
		ckList.block<1,3>(i,0) = ckMin.transpose();
        errork.push_back(errorMin);
    }
	
	Freg = hornRegistration(dkList,ckList);
}


/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh.
///
/// @param[out] dkList location of Atip in fiducial B body coordinates, n x 3 matrix of transposed vectors
/// @param[in]  vertices list of vertices on mesh, corresponding to bone surface
/// @param[in]  vertexTriangleNeighborIndex list of 1x6 vectors. First 3 Elements are indices into vertices list, Second 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
void multiStepIcpPointMeshRegistration(
                              const Eigen::MatrixXd& dkList,
                              const std::vector<Eigen::Vector3d>& vertices,
                              const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                                       Eigen::Affine3d& Freg,
                                       Eigen::MatrixXd& skList,
                                       Eigen::MatrixXd& ckList,
                              std::vector<double>& errork){

    bool debug = true;
    Freg.setIdentity();
	for(int i = 0; i < 300; ++i){
        
        if(debug) std::cout << "\n\nFreg before iteration " << i << ":\n\n" << Freg.matrix() << "\n\n";
		icpPointMeshRegistration(dkList,vertices,vertexTriangleNeighborIndex,Freg,skList,ckList,errork);
	}
}




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
template<typename RTREE>
void optimizedICPStep(
                      const Eigen::MatrixXd& dkList,
                              const RTREE& rtree,
                      Eigen::Affine3d& Freg,
                      Eigen::MatrixXd& skList,
                      Eigen::MatrixXd& ckList,
                              std::vector<double>& errork){
    
    errork.clear();
    ckList.resize(dkList.rows(),3);
    skList.resize(dkList.rows(),3);
    
    for (int i=0; i<dkList.rows(); ++i){
        double errorMin=std::numeric_limits<double>::max();
        Eigen::Vector3d ckMin;
        Eigen::Vector3d dk_i(dkList.block<1,3>(i,0).transpose());
        
        Eigen::Vector3d sk(Freg*dk_i);
        skList.block<1,3>(i,0) = sk.transpose();
        
        // starting query size will affect performance
        /// @todo consider making querySize a parameter
        int querySize = 4;
        bool closestFound = false;
        while(!closestFound){
            // If the distance to the next box was greater than to the nearest triangle
            // or the distance to the triangle was equal to 0
            // break the loop.
            
            // find 2 nearest values to a point
            std::vector<cis::value> result_n;
            rtree.query(bgi::nearest(sk, querySize), std::back_inserter(result_n));
            
            double prevDistance=std::numeric_limits<double>::max();
            for(int i = 0; i < result_n.size(); ++i){
                /// @todo may need to compare errorTemp to the distance from the point to the box instead of to the polygon.
                Eigen::Vector3d ckTemp = FindClosestPoint(sk, result_n[i].second.outer()[0], result_n[i].second.outer()[1], result_n[i].second.outer()[2]);
                double errorTemp = (sk-ckTemp).norm();
                
                // leave the loop if the distance to the box is larger than the distance to the polygon
                double distance = bg::distance(sk,result_n[i].first);
                if( errorTemp > prevDistance || distance ==0){
                    closestFound = true;
                    break;
                }
                
                
                if (errorTemp < errorMin){
                    // put dk (qk)  into A put ck into B
                    ckMin = ckTemp;
                    errorMin = errorTemp;
                }
                
                prevDistance = distance;
            }
            
            // increase the query size and rerun it if we didn't find the closest point for certain
            querySize *=2;
        }
        
        ckList.block<1,3>(i,0) = ckMin.transpose();
        errork.push_back(errorMin);
    }
    
    Freg = hornRegistration(dkList,ckList);
}


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
void optimizedICP(
                  const Eigen::MatrixXd& dkList,
                  const std::vector<Eigen::Vector3d>& vertices,
                  const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                  Eigen::Affine3d& Freg,
                  Eigen::MatrixXd& skList,
                  Eigen::MatrixXd& ckList,
                  std::vector<double>& errork){
    
    bool debug = true;
    Freg.setIdentity();
    
    
    // polygons
    std::vector<cis::polygon> polygons;
    
    
    // create some polygons
    for ( auto&& triangle : vertexTriangleNeighborIndex )
    {
        // create a polygon
        cis::polygon p;
        
        Eigen::Vector3d p1 = vertices[triangle(0)];
        Eigen::Vector3d p2 = vertices[triangle(1)];
        Eigen::Vector3d p3 = vertices[triangle(2)];
        p.outer().push_back(p1);
        p.outer().push_back(p2);
        p.outer().push_back(p3);
        
        // add polygon
        polygons.push_back(p);
    }
    
    // create the rtree using default constructor
    bgi::rtree< cis::value, bgi::rstar<16, 4> > rtree;
    
    // fill the spatial index
    for ( auto && polygon : polygons )
    {
        
        // calculate polygon bounding box
        cis::box b = bg::return_envelope<cis::box>(polygon);
        // insert new value
        rtree.insert(std::make_pair(b, polygon));
    }
    
    for(int i = 0; i < 1000; ++i){
        
        if(debug) std::cout << "\n\nFreg before iteration " << i << ":\n\n" << Freg.matrix() << "\n\n";
        optimizedICPStep(dkList,rtree,Freg,skList,ckList,errork);
    }
}

#endif // _PA3_HPP_