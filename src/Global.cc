#include "Global.h"

namespace Global
{
    std::string frameWindow = "ORB-SLAM: Frame Viewer";
    std::string mapWindow = "ORB-SLAM3: Map Viewer";
    int frameWindowWidth = 600;
    int frameWindowHeight = 350;


    //converts row major to column major, i.e gives transpose
    //first 3 cloumns represent orientation and last one represent position
    pangolin::OpenGlMatrix ToOpenGlMatrix(pangolin::OpenGlMatrix &M, const Eigen::Matrix3f &ori, const Eigen::Vector3f &pos)
    {
        M.SetIdentity();

        // Fill rotation (column-major layout)
        M.m[0] = ori(0, 0); M.m[4] = ori(1, 0); M.m[8]  = ori(2, 0);
        M.m[1] = ori(0, 1); M.m[5] = ori(1, 1); M.m[9]  = ori(2, 1);
        M.m[2] = ori(0, 2); M.m[6] = ori(1, 2); M.m[10] = ori(2, 2);

        // Fill translation
        M.m[12] = pos(0);
        M.m[13] = pos(1);
        M.m[14] = pos(2);

        return M;
    }

    void PrintOpenGlMatrix(const pangolin::OpenGlMatrix& mat) {
    std::cout << "OpenGlMatrix:" << std::endl;
    for(int row = 0; row < 4; ++row) {
        for(int col = 0; col < 4; ++col) {
            std::cout << mat.m[col * 4 + row] << " ";
        }
        std::cout << std::endl;
    }
    }
}
