#ifndef _ITERATIVE_CLOSEST_POINT_HPP_
#define _ITERATIVE_CLOSEST_POINT_HPP_


#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/math/special_functions/fpclassify.hpp>


#include <boost/geometry/index/rtree.hpp>

#include <cmath>
#include <vector>
#include <iostream>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;



#include "hornRegistration.hpp"
#include "TerminationCriteria.hpp"

/// Finds the nearest point on a line segment to a point in space.  Called by the function OutsideOfTriangle()
/// to determine where the nearest point is to each side of the triangle.
/// @param a is the point in space
/// @param p is one end of the line segment represented by an Eigen::Vector3d
/// @param q is the other end of the line segment represented by an Eigen::Vector3d
/// @see page 8 of PointPairs.pdf from the notes
Eigen::Vector4d ProjectOnSegment(const Eigen::Vector3d& c, const Eigen::Vector3d& p, const Eigen::Vector3d& q)
{
    Eigen::Vector3d pMinusQ = q - p;
    double lambda = (c-p).dot(pMinusQ)/(pMinusQ.dot(pMinusQ));
    double zero = 0.0;
    double one = 1.0;
    double vertexlogic = one;
    if (lambda <= 0 || lambda >= 1){
        vertexlogic = zero;
    }
    lambda = std::max(zero,std::min(lambda,one));
    Eigen::Vector4d cNew;
    cNew.block<3,1>(0,0) = p+lambda*pMinusQ;
    cNew(3) = vertexlogic;
    return cNew;
}

/// Determines if two points are equal.  Used by the function OutsideOfTriangle to determine if the nearest
/// point on the triangle lies on a vertice
/// @param a is the point in space
/// @param p is the first vertice of the triangle represented by an Eigen::Vector3d
bool PointEqualityCheck(const Eigen::Vector3d& a, const Eigen::Vector3d& b){
    bool tf = (a(0) == b(0) && a(1) == b(1) && a(2) == b(2));
    return tf;
}

/// Finds the closest point on the triangle to a point in space if the closest point lies on an edge or vertice
/// @param a is the point in space
/// @param p is the first vertice of the triangle represented by an Eigen::Vector3d
/// @param q is the second vertice of the triangle represented by an Eigen::Vector3d
/// @param r is the third vertice of the triangle represented by an Eigen::Vector3d
/// @see page 8 of PointPairs.pdf from the notes
Eigen::Vector3d OutsideOfTriangle(const Eigen::Vector3d& a, const Eigen::Vector3d& p, const Eigen::Vector3d& q, const Eigen::Vector3d& r)
{
    Eigen::MatrixXd c(3,4);
    c.block<1,4>(0,0) = ProjectOnSegment(a,r,p).transpose();
    c.block<1,4>(1,0) = ProjectOnSegment(a,p,q).transpose();
    c.block<1,4>(2,0) = ProjectOnSegment(a,q,r).transpose();
    Eigen::Vector3d cnew;
    if (PointEqualityCheck(c.block<1,3>(0,0),c.block<1,3>(1,0))) cnew = c.block<1,3>(0,0);
    else if (PointEqualityCheck(c.block<1,3>(0,0),c.block<1,3>(2,0))) cnew = c.block<1,3>(0,0);
    else if (PointEqualityCheck(c.block<1,3>(1,0),c.block<1,3>(2,0))) cnew = c.block<1,3>(1,0);
    else{
        for (int i=0; i<3; i++){
            if (c(i,3) == 1){
                cnew = c.block<1,3>(i,0);
            }
        }
    }
    return cnew;
}

/// Finds the closest point on the triangle to a point in space.  If the closest point lies with in triangle,
/// then the function finds the nearest point internally.  Else if the closest point lies on an edge or vertice,
/// the function OutsideOfTriangle() is called to find the nearest point
/// @param a is the point in space represented by an Eigen::Vector3d
/// @param vertices are the three vertices of the triangle represented by standard vector of Eigen::Vector3d
/// @see page 7 of PointPairs.pdf from the notes
Eigen::Vector3d FindClosestPoint(const Eigen::Vector3d& p, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3)
{
    Eigen::Vector3d u = p2-p1;
    Eigen::Vector3d v = p3-p1;
    Eigen::Vector3d w = p-p1;
    Eigen::Vector3d n = u.cross(v);
    double gamma = u.cross(w).dot(n)/(n.dot(n));
    double beta = w.cross(v).dot(n)/(n.dot(n));
    double alpha = 1-gamma-beta;
    Eigen::Vector3d projectedPoint;
    if ((alpha < 0 || alpha > 1) || (beta < 0 || beta > 1) || (gamma < 0 || gamma > 1)){
        projectedPoint = OutsideOfTriangle(p,p1,p2,p3);
    }
    else projectedPoint = alpha*p1+beta*p2+gamma*p3;

    return projectedPoint;
}

Eigen::Vector3d FindClosestPoint(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& vertices, const Eigen::VectorXd& triangle) {
    
    Eigen::Vector3d p1 = vertices[triangle(0)];
    Eigen::Vector3d p2 = vertices[triangle(1)];
    Eigen::Vector3d p3 = vertices[triangle(2)];

    return FindClosestPoint(p,p1,p2,p3);
}




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
void ICPwithSimpleSearchStep(
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
/// prior known body data, and a triangle mesh. Uses brute force iteration.
///
/// @param[in] dkList location of Atip in fiducial B body coordinates, n x 3 matrix of transposed vectors
/// @param[in]  vertices list of vertices on mesh, corresponding to bone surface
/// @param[in]  vertexTriangleNeighborIndex list of 1x6 vectors. First 3 Elements are indices into vertices list, Second 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @param[in,out] terminationCriteria  Collects statistics and determines when algorithm should stop. C++ concept matches TerminationCriteria class.
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
template<typename TerminationType = TerminationCriteria>
void ICPwithSimpleSearch(
                              const Eigen::MatrixXd& dkList,
                              const std::vector<Eigen::Vector3d>& vertices,
                              const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                              TerminationType& terminationCriteria,
                              Eigen::Affine3d& Freg,
                              Eigen::MatrixXd& skList,
                              Eigen::MatrixXd& ckList,
                              std::vector<double>& errork,
                              bool debug = false){

    Freg.setIdentity();
    
    for(int i = 0; !terminationCriteria.shouldTerminate(); i++){
        
        if(debug) std::cout << "\n\nFreg before iteration " << i << ":\n\n" << Freg.matrix() << "\n\n";
        
        if(i) terminationCriteria.nextIteration();
		ICPwithSimpleSearchStep(dkList,vertices,vertexTriangleNeighborIndex,Freg,skList,ckList,errork);
        
        for(auto && err : errork){
            terminationCriteria(err);
        }
        
	}
}




/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh. Uses a spatial index to accelerate
/// nearest neighbor lookup of triangles.
///
/// @tparam boost::geometry::rtree data structure type configured with the triangle set
///
/// @param[in] dkList location of Atip in fiducial B body coordinates, n x 3 matrix of transposed vectors
/// @param[in] vertices list of vertices on mesh, corresponding to bone surface
/// @param[in] rtree actual triangle data index
/// @param[in,out] terminationCriteria  Collects statistics and determines when algorithm should stop. C++ concept matches TerminationCriteria class.
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
/// @param[out] errork norm between ck and dk
template<typename RTREE>
void ICPwithSpatialIndexStep(
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
        double minErrorAKAdistanceToClosestTriangle=std::numeric_limits<double>::max();
        Eigen::Vector3d ckClosestPointOnMesh;
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
            
            // results should be listed in order from closest box to furthest box,
            // but the boxes can overlap so untl the closest point on the nearest triangle
            // is closer than boxes later in the list, there may be closer triangle points
            // than the current one.
            for(auto&& result : result_n){
                /// @todo may need to compare errorTemp to the distance from the point to the box instead of to the polygon.
                Eigen::Vector3d ckClosestPointOnCurrentTriangle = FindClosestPoint(sk, result.second.outer()[0], result.second.outer()[1], result.second.outer()[2]);
                double distanceToCurrentBox = bg::distance(sk,result.first);
                double distanceToCurrentTriangle = bg::distance(sk,ckClosestPointOnCurrentTriangle);
                
                
                // leave the loop if the distance to the box is larger than the distance to the polygon
                if( minErrorAKAdistanceToClosestTriangle < distanceToCurrentBox || distanceToCurrentTriangle == 0) closestFound = true;
                
                
                if (distanceToCurrentTriangle < minErrorAKAdistanceToClosestTriangle){
                    // put dk (qk)  into A put ck into B
                    ckClosestPointOnMesh = ckClosestPointOnCurrentTriangle;
                    minErrorAKAdistanceToClosestTriangle = distanceToCurrentTriangle;
                }
                
            }
            
            // increase the query size and rerun it if we didn't find the closest point for certain
            querySize *=2;
        }
        
        ckList.block<1,3>(i,0) = ckClosestPointOnMesh.transpose();
        BOOST_VERIFY(!boost::math::isnan(minErrorAKAdistanceToClosestTriangle));
        errork.push_back(minErrorAKAdistanceToClosestTriangle);
    }
    
    Freg = hornRegistration(dkList,ckList);
}


/// perform ICPregistration on source data consisting of sensor data,
/// prior known body data, and a triangle mesh. Uses a spatial index for accessing triangles.
///
/// @param[in] dkList location of Atip in fiducial B body coordinates, n x 3 matrix of transposed vectors
/// @param[in]  vertices list of vertices on mesh, corresponding to bone surface
/// @param[in]  vertexTriangleNeighborIndex list of 1x6 vectors. First 3 Elements are indices into vertices list, Second 3 correspond to neighbors. -1 indicates not a neighbor. List of triangles on mesh, corresponding to bone surface.
/// @param[in,out] terminationCriteria  Collects statistics and determines when algorithm should stop. C++ concept matches TerminationCriteria class.
/// @param[out] ck location of CT mesh closest to sample points, nx3 matrix of transposed vectors
/// @param[out] errork norm between ck and dk
template<typename TerminationType = TerminationCriteria>
void ICPwithSpatialIndex(
                  const Eigen::MatrixXd& dkList,
                  const std::vector<Eigen::Vector3d>& vertices,
                  const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                  TerminationType& terminationCriteria,
                  Eigen::Affine3d& Freg,
                  Eigen::MatrixXd& skList,
                  Eigen::MatrixXd& ckList,
                  std::vector<double>& errork,
                  bool debug = false){
    
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
    
    for(int i = 0; !terminationCriteria.shouldTerminate(); i++){
        
        if(debug) std::cout << "\n\nFreg before iteration " << i << ":\n\n" << Freg.matrix() << "\n\n";
        if(i) terminationCriteria.nextIteration();
        ICPwithSpatialIndexStep(dkList,rtree,Freg,skList,ckList,errork);
        
        for(auto && err : errork){
            terminationCriteria(err);
        }
    }
}


#endif // _ITERATIVE_CLOSEST_POINT_HPP_
