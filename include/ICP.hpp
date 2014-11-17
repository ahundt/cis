#ifndef _ICP_HPP_
#define _ICP_HPP_

Eigen::Vector4d ProjectOnSegment(Eigen::Vector3d c, Eigen::Vector3d p, Eigen::Vector3d q)
{
    std::cout << "\n\ntest1\n\n";
    Eigen::Vector3d pMinusQ = q - p;
    double lambda = (c-p).dot(pMinusQ)/(pMinusQ.dot(pMinusQ));
    double zero = 0.0;
    double one = 1.0;
    double vertexlogic = one;
    if (lambda <= 0 || lambda >= 1){
        vertexlogic = zero;
    }
    std::cout << "\n\ntest2\n\n";
    lambda = std::max(zero,std::min(lambda,one));
    Eigen::Vector4d cNew;
    std::cout << "\n\ntest3\n\n";
    cNew.block<3,1>(0,0) = p+lambda*pMinusQ;
    cNew(3) = vertexlogic;
    std::cout << cNew;
    return cNew;
}

Eigen::Vector3d FindClosestPoint(Eigen::Vector3d a, std::vector<Eigen::Vector3d>& vertices)
{
    Eigen::Vector3d p = vertices[1];
    Eigen::Vector3d q = vertices[2];
    Eigen::Vector3d r = vertices[3];
    Eigen::MatrixXd c(3,4);
    c.block<1,4>(0,0) = ProjectOnSegment(a,r,p).transpose();
    c.block<1,4>(1,0) = ProjectOnSegment(a,p,q).transpose();
    c.block<1,4>(2,0) = ProjectOnSegment(a,q,r).transpose();
    double vertexlogic = c.col(3).sum();
    Eigen::Vector3d cnew;

    if (vertexlogic == 0) cnew = c.block<3,3>(0,0).colwise().mean();
    if (vertexlogic == 2){
        for (int i; i<3; i++){
            if (c(i,3) == 1) cnew = c.block<1,3>(0,0);
        }
    }
    if (vertexlogic == 3){
        if (c(0,0) == c(1,0)) cnew = c.block<1,3>(0,0);
        else if (c(0,0) == c(2,0)) cnew = c.block<1,3>(0,0);
        else cnew = c.block<1,3>(1,0);
    }
    return cnew;
}


#endif // _ICP_HPP_
