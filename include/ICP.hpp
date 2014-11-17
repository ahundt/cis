#ifndef _ICP_HPP_
#define _ICP_HPP_

Eigen::Vector4d ProjectOnSegment(Eigen::Vector3d c, Eigen::Vector3d p, Eigen::Vector3d q)
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

bool PointEqualityCheck(Eigen::Vector3d a, Eigen::Vector3d b){
    bool tf = (a(0) == b(0) && a(1) == b(1) && a(2) == b(2));
    return tf;
}

Eigen::Vector3d OutsideOfTriangle(Eigen::Vector3d a, Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r)
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


Eigen::Vector3d FindClosestPoint(Eigen::Vector3d p, std::vector<Eigen::Vector3d>& vertices)
{
    Eigen::Vector3d p1 = vertices[1];
    Eigen::Vector3d p2 = vertices[2];
    Eigen::Vector3d p3 = vertices[3];
    Eigen::Vector3d u = p2-p1;
    Eigen::Vector3d v = p3-p1;
    Eigen::Vector3d w = p-p1;
    Eigen::Vector3d n = u.cross(v);
    double gamma = u.cross(w).dot(n)/(n.dot(n));
    double beta = w.cross(v).dot(n)/(n.dot(n));
    double alpha = 1-gamma-beta;
    Eigen::Vector3d projectedPoint;
    if ((alpha <= 0 || alpha >= 1) || (beta <= 0 || beta >= 1) || (gamma <= 0 || gamma >= 1)){
        projectedPoint = OutsideOfTriangle(p,p1,p2,p3);
    }
    else projectedPoint = alpha*p1+beta*p2+gamma*p3;

    return projectedPoint;
}


#endif // _ICP_HPP_
