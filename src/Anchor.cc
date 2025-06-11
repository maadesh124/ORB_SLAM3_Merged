
#include "Anchor.h"

namespace ORB_SLAM3
{
    
std::vector<Eigen::Vector3f> Anchor::getRefPositions(){
    vector<MapPoint*> vMp=this->refs;
    std::vector<Eigen::Vector3f> vPos;
    for(MapPoint* mp:vMp){
        Eigen::Vector3f t1=mp->GetWorldPosEigen();
       // Eigen::Vector3f t2(t1[0],t1[1],t1[2]);
        vPos.push_back(t1);
    }

    return vPos;

}

Eigen::Matrix3f Anchor::solveOrthogonalProcrustes(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> Y) {
    // Ensure input vectors have the same size
    if (X.size() != Y.size()) {
        throw std::invalid_argument("Input and output vectors must have the same size.");
    }

    size_t N = X.size(); // Number of vectors

    // Compute the centroids of X and Y
    Eigen::Vector3f centroidX = Eigen::Vector3f::Zero();
    Eigen::Vector3f centroidY = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        centroidX += X[i];
        centroidY += Y[i];
    }
    centroidX /= N;
    centroidY /= N;

    // Center the input vectors
    Eigen::MatrixXf Xc(3, N), Yc(3, N);
    for (size_t i = 0; i < N; ++i) {
        Xc.col(i) = X[i] - centroidX;
        Yc.col(i) = Y[i] - centroidY;
    }

    // Compute the cross-covariance matrix H
    Eigen::Matrix3f H = Xc * Yc.transpose();

    // Perform Singular Value Decomposition (SVD)
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    // Compute the rotation matrix R
    Eigen::Matrix3f R = V * U.transpose();

    // Ensure det(R) = 1 (proper rotation matrix)
    if (R.determinant() < 0) {
        V.col(2) *= -1; // Flip the sign of the last column of V
        R = V * U.transpose();
    }

    return R;
}

Eigen::Vector3f Anchor::update(long unsigned int mapId){

if(this->prevRefPos.size()==0)
return this->pos;

if(mapId!=this->map->GetId())
return this->pos;

if(this->prevRefPos.size()!=this->refs.size())
std::cerr<<"No. of refs varies"<<endl;

int i=0;
double wi=1,swi=0;
Eigen::Vector3f c(0,0,0);
for(Eigen::Vector3f pp:this->prevRefPos){
    wi=1/(pp-this->pos).norm();
    Eigen::Vector3f cp=this->refs.at(i)->GetWorldPosEigen();
    Eigen::Vector3f ci=pp-this->pos-cp;
    c=c+(wi*ci);
    swi=swi+wi;

    i++;


}
this->pos=-1*c/swi;

std::vector<Eigen::Vector3f> alignment;
std::vector<Eigen::Vector3f> curPos;
i=0;
for(Eigen::Vector3f pp:this->prevRefPos){
    alignment.push_back(this->ori*(this->pos-pp).normalized());
    curPos.push_back((this->pos-this->refs.at(i)->GetWorldPosEigen()).normalized());
    i++;
}

Eigen::Matrix3f R=solveOrthogonalProcrustes(curPos,alignment);
this->ori.row(0)=R.row(0).normalized();
this->ori.row(1)=R.row(1).normalized();
this->ori.row(2)=R.row(2).normalized();

return this->pos;
}





Eigen::Vector3f Anchor::update1(long unsigned int mapId){
if(mapId!=this->map->GetId())
return this->pos;

if(this->prevRefPos.size()!=this->refs.size())
std::cerr<<"No. of refs varies"<<endl;



std::vector<Eigen::Vector3f> alignment;
std::vector<Eigen::Vector3f> curPos;


 int i=0;
for(Eigen::Vector3f pp:this->prevRefPos){

    pp=pp.normalized();
    alignment.push_back(this->ori*pp);
    curPos.push_back(this->refs.at(i)->GetWorldPosEigen().normalized());
   i++; 
}
Eigen::Matrix3f R=solveOrthogonalProcrustes(curPos,alignment);


this->ori.row(0)=R.row(0).normalized();
this->ori.row(1)=R.row(1).normalized();
this->ori.row(2)=R.row(2).normalized();

return this->pos;
}


}