


#ifndef ANCHOR_H
#define ANCHOR_H
#include "Map.h"
#include "MapPoint.h"
// #include <vector>

namespace ORB_SLAM3{
class Map;

class Anchor{
public:
Map* map;
std::vector<MapPoint*> refs;
std::vector<Eigen::Vector3f> prevRefPos;//position of refs when previous frame was processed
Eigen::Vector3f pos;//position of anchor
Eigen::Matrix3f ori;//orientation of anchor-rotation matrix 1st row indicates direction of x axis,2nd row indicates direction of y ais ...
//updates position  and orientation of anchor; 
Eigen::Vector3f update(long unsigned int mapId);
//returns current position of all ref points
std::vector<Eigen::Vector3f> getRefPositions();
size_t anchorId;

//how much each position vector aligns with three orientation vectors defines the 
//orientation relationship between a reference point and anchor.
//this alignment(varies from 0 to 1) is given by dot product when both vector is unity.
//alignment=ori(unit vector)*(anchor pos-ref pos)(unit vector);this alignment vector is 3x1 vector with
//each row indicating ref point's alignment with x,y and z axis.
//during next frame processing references move.So anchors orientation must be such that 
//squared alignment error is minimum.alignent error =sum of ||cur alignment - prev alignment||
    


//gives solution to orthogonal procrustes problem
Eigen::Matrix3f solveOrthogonalProcrustes(const std::vector<Eigen::Vector3f> X, const std::vector<Eigen::Vector3f> Y);
//updates only orientation
Eigen::Vector3f update1(long unsigned int mapId) ;
//void setPrevRefPos();

};
}

#endif
