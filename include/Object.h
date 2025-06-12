#ifndef OBJECT_H
#define OBJECT_H

#include <pangolin/pangolin.h>
#include <pangolin/gl/glsl.h>
#include <vector>
#include <string>
#include "Anchor.h"
#include <boost/serialization/vector.hpp>





namespace ORB_SLAM3
{
class Map;
class Anchor;
// Object class: loads and renders a mesh from OBJ
class Object {

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & scaleFactor;
        ar & pose;
        ar & anchor_id;
    }

public:
    Object();

    Map* map;
    Anchor *anchor;
    size_t objectId;
    Eigen::Vector3f pos;//relative position of obj with respect to anchor
    Eigen::Matrix3f ori;//relative orientation of obj with respect to anchor
    size_t anchor_id;


    vector<float> pose;
    float scaleFactor=1.0;

    
    void PreSave();
    void PostLoad();

    // Load mesh from OBJ file
    bool LoadFromObj(const std::string& filename);

    // Upload mesh data to GPU
    void UploadToGPU();

    // Render the mesh
    void Draw() const;


    struct Vertex {
        float x, y, z;
        float nx, ny, nz;
    };

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    pangolin::GlBuffer vbo, ibo;
    size_t index_count;


};
}
#endif