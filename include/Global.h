#ifndef GLOBAL_H
#define GLOBAL_H

#include <string>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace Global
{
    extern std::string frameWindow;
    extern std::string mapWindow;
    extern int frameWindowWidth;
    extern int frameWindowHeight;

    
    void PrintOpenGlMatrix(const pangolin::OpenGlMatrix& mat) ;
    pangolin::OpenGlMatrix ToOpenGlMatrix(pangolin::OpenGlMatrix &M, const Eigen::Matrix3f &ori, const Eigen::Vector3f &pos);
    void ExtractPoseComponents(const pangolin::OpenGlMatrix& pose,
                           Eigen::Vector3f* position,
                           Eigen::Matrix3f* orientation);
                               
}

#endif
