#ifndef _ICP_HPP_
#define _ICP_HPP_

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
Eigen::Vector3d FindClosestPoint(const Eigen::Vector3d& p, const std::vector<Eigen::Vector3d>& vertices, const Eigen::VectorXd& triangle)
{
    Eigen::Vector3d p1 = vertices[triangle(0)];
    Eigen::Vector3d p2 = vertices[triangle(1)];
    Eigen::Vector3d p3 = vertices[triangle(2)];
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
/// @param[out] dk location of Atip in fiducial B body coordinates
/// @param[out] ck location of CT mesh closest to sample points
/// @param[out] errork norm between ck and dk
void icpPointMeshRegistration(const std::vector<Eigen::MatrixXd>& NA,
                              const std::vector<Eigen::MatrixXd>& NB,
                              const Eigen::Vector3d& Atip,
                              const Eigen::MatrixXd& bodyAmarkerLEDs,
                              const Eigen::MatrixXd& bodyBmarkerLEDs,
                              const std::vector<Eigen::Vector3d>& vertices,
                              const std::vector<Eigen::VectorXd>& vertexTriangleNeighborIndex,
                              std::vector<Eigen::Vector3d>& dk,
                              std::vector<Eigen::Vector3d>& ck,
                              std::vector<double>& errork){
    
    Eigen::Vector3d ckMin;
    Eigen::Affine3d Freg;
    Freg.setIdentity();
    
    for (int i=0; i<NA.size(); i++){
        double errorMin=std::numeric_limits<double>::max();
        Eigen::Affine3d FaAffine(hornRegistration(bodyAmarkerLEDs,NA[i])); // a: PA3-A-Debug-SampleReadingsTest A: Problem3-BodyA
        Eigen::Affine3d FbInverseAffine(hornRegistration(NB[i], bodyBmarkerLEDs)); // b: PA3-A-Debug-SampleReadingsTest B: Problem3-BodyB
        dk.push_back(Eigen::Vector3d(FbInverseAffine*FaAffine*Atip)); // Atip: Problem3-BodyA (last line)
        Eigen::Vector3d sk = Freg*dk[i];
        for (auto&& triangle : vertexTriangleNeighborIndex){
            Eigen::Vector3d ckTemp = FindClosestPoint(sk, vertices, triangle);
            double errorTemp = (ckTemp-dk[i]).norm();
            if (errorTemp < errorMin){
                ckMin = ckTemp;
                errorMin = errorTemp;
            }
        }
        ck.push_back(ckMin);
        errork.push_back(errorMin);
    }
}

#endif // _ICP_HPP_
