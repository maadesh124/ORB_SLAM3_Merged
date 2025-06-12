/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include "Object.h"
#include "Global.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3
{


MapDrawer::MapDrawer(Atlas* pAtlas, const string &strSettingPath):mpAtlas(pAtlas)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    bool is_correct = ParseViewerParamFile(fSettings);

    if(!is_correct)
    {
        std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
        try
        {
            throw -1;
        }
        catch(exception &e)
        {

        }
    }
}

void MapDrawer::DrawReferenceAxes()
    {


        glPushMatrix();

        // Draw X axis (Red)
        glLineWidth(3.0f); 
        glColor3f(1.0, 0.0, 0.0); // Red
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0); // Origin
        glVertex3f(1.0, 0.0, 0.0); // X axis end
        glEnd();

        // Draw Y axis (Green)
        glLineWidth(3.0f); 
        glColor3f(0.0, 1.0, 0.0); // Green
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0); // Origin
        glVertex3f(0.0, 1.0, 0.0); // Y axis end
        glEnd();

        // Draw Z axis (Blue)
        glLineWidth(3.0f); 
        glColor3f(0.0, 0.0, 1.0); // Blue
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0); // Origin
        glVertex3f(0.0, 0.0, 1.0); // Z axis end
        glEnd();

        glPopMatrix();

          // Set flag to indicate axes have been drawn
    }



void MapDrawer::DrawAnchors()
{
    pangolin::BindToContext(Global::mapWindow);
    Map* curMap=mpAtlas->GetCurrentMap();
    glPushMatrix();
    for(Anchor* anchor:curMap->anchors)
    {
    glLineWidth(2.0f); // Set the line width
    glColor3f(1.0, 0.0, 0.0); // Set the vector color (green in this case)
    glBegin(GL_LINES);
    glVertex3f(anchor->pos[0],anchor->pos[1],anchor->pos[2] ); // Start point
    Eigen::Vector3f endPoint=anchor->pos + anchor->ori.row(0).transpose();//=anchor->pos+ anchor->ori.row(0);
    glVertex3f(endPoint[0], endPoint[1], endPoint[2]);       // End point
    glEnd();


    glLineWidth(2.0f); // Set the line width
    glColor3f(0.0, 1.0, 0.0); // Set the vector color (green in this case)
    glBegin(GL_LINES);
    glVertex3f(anchor->pos[0],anchor->pos[1],anchor->pos[2] ); // Start point
    endPoint=anchor->pos + anchor->ori.row(1).transpose();//=anchor->pos+ anchor->ori.row(0);
    glVertex3f(endPoint[0], endPoint[1], endPoint[2]);       // End point
    glEnd();


    glLineWidth(2.0f); // Set the line width
    glColor3f(0.0, 0.0, 1.0); // Set the vector color (green in this case)
    glBegin(GL_LINES);
    glVertex3f(anchor->pos[0],anchor->pos[1],anchor->pos[2] ); // Start point
    endPoint=anchor->pos + anchor->ori.row(2).transpose();//=anchor->pos+ anchor->ori.row(0);
    glVertex3f(endPoint[0], endPoint[1], endPoint[2]);       // End point
    glEnd();


    }

    glPopMatrix();

   
}

void MapDrawer::DrawCubes()
{
    cout<<" no. of objects"<<mpAtlas->GetCurrentMap()->objects.size()<<endl;
        for(Object* object: mpAtlas->GetCurrentMap()->objects)
        {
        pangolin::OpenGlMatrix model;
        Eigen::Matrix3f globOri=object->anchor->ori*object->ori;
        Eigen::Vector3f globPos=object->pos + object->anchor->pos;
        model=Global::ToOpenGlMatrix(model,globOri,globPos);

        model.m[0] *= object->scaleFactor; // scale X
        model.m[5] *= object->scaleFactor; // scale Y
        model.m[10] *= object->scaleFactor; // scale Z

        glPushMatrix();
        glMultMatrixd(model.m);
        pangolin::glDrawColouredCube();
        glPopMatrix();

        }
}

void MapDrawer::DrawObjects( pangolin::GlSlProgram& shader ,pangolin::OpenGlMatrix view,pangolin::OpenGlMatrix proj)
{


    Map* curMap=mpAtlas->GetCurrentMap();

    //  cout<<"n obs in MapDrawer"<<mpAtlas->GetCurrentMap()->objects.size()<<endl;
    //  cout<<" mp id in MapDrawer"<<mpAtlas->GetCurrentMap()->GetId();
      
    pangolin::BindToContext(Global::mapWindow);
    glClear(GL_DEPTH_BUFFER_BIT);
    pangolin::OpenGlMatrix model;
    int i=0;
    shader.Bind();
    for(Object* object:curMap->objects)
    {
        //cout<<"Object rendered in mp drawer"<<i++<<endl;
        Eigen::Matrix3f globOri=object->anchor->ori*object->ori;
        Eigen::Vector3f globPos=object->pos + object->anchor->pos;
        model=Global::ToOpenGlMatrix(model,globOri,globPos);

        model.m[0] *= object->scaleFactor; // scale X
        model.m[5] *= object->scaleFactor; // scale Y
        model.m[10] *= object->scaleFactor; // scale Z

        shader.SetUniform("model", model);
        shader.SetUniform("view", view);
        shader.SetUniform("projection", proj);
        //pangolin::glDrawColouredCube();
        object->Draw();
    }
    shader.Unbind();
}




bool MapDrawer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;

    cv::FileNode node = fSettings["Viewer.KeyFrameSize"];
    if(!node.empty())
    {
        mKeyFrameSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.KeyFrameLineWidth"];
    if(!node.empty())
    {
        mKeyFrameLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.KeyFrameLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.GraphLineWidth"];
    if(!node.empty())
    {
        mGraphLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.GraphLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.PointSize"];
    if(!node.empty())
    {
        mPointSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.PointSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraSize"];
    if(!node.empty())
    {
        mCameraSize = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraSize parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.CameraLineWidth"];
    if(!node.empty())
    {
        mCameraLineWidth = node.real();
    }
    else
    {
        std::cerr << "*Viewer.CameraLineWidth parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
            unsigned int index_color = pKF->mnOriginMapId;

            if (index_color > 5)
                index_color = 0;

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            if(!pKF->GetParent()) // It is the first KF in the map
            {
                glLineWidth(mKeyFrameLineWidth*5);
                glColor3f(1.0f,0.0f,0.0f);
                glBegin(GL_LINES);
            }
            else
            {
                glLineWidth(mKeyFrameLineWidth);
                glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                glBegin(GL_LINES);
            }

            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

            glEnd();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }

    if(bDrawInertialGraph && mpAtlas->isImuInitialized())
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(1.0f,0.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        //Draw inertial links
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKFi = vpKFs[i];
            cv::Mat Ow = pKFi->GetCameraCenter();
            KeyFrame* pNext = pKFi->mNextKF;
            if(pNext)
            {
                cv::Mat Owp = pNext->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }
        }

        glEnd();
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();

    if(bDrawKF)
    {
        for(Map* pMap : vpMaps)
        {
            if(pMap == mpAtlas->GetCurrentMap())
                continue;

            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();
                unsigned int index_color = pKF->mnOriginMapId;

                if (index_color > 5)
                    index_color = 0;

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                if(!vpKFs[i]->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth*5);
                    glColor3f(1.0f,0.0f,0.0f);
                    glBegin(GL_LINES);
                }
                else
                {
                    glLineWidth(mKeyFrameLineWidth);
                    glColor3f(mfFrameColors[index_color][0],mfFrameColors[index_color][1],mfFrameColors[index_color][2]);
                    glBegin(GL_LINES);
                }

                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
    }
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw, pangolin::OpenGlMatrix &MTwwp)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        cv::Mat Rwwp(3,3,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;

        MOw.SetIdentity();
        MOw.m[12] = twc.at<float>(0);
        MOw.m[13] = twc.at<float>(1);
        MOw.m[14] = twc.at<float>(2);

        MTwwp.SetIdentity();
        MTwwp.m[0] = Rwwp.at<float>(0,0);
        MTwwp.m[1] = Rwwp.at<float>(1,0);
        MTwwp.m[2] = Rwwp.at<float>(2,0);

        MTwwp.m[4] = Rwwp.at<float>(0,1);
        MTwwp.m[5] = Rwwp.at<float>(1,1);
        MTwwp.m[6] = Rwwp.at<float>(2,1);

        MTwwp.m[8] = Rwwp.at<float>(0,2);
        MTwwp.m[9] = Rwwp.at<float>(1,2);
        MTwwp.m[10] = Rwwp.at<float>(2,2);

        MTwwp.m[12] = twc.at<float>(0);
        MTwwp.m[13] = twc.at<float>(1);
        MTwwp.m[14] = twc.at<float>(2);
    }
    else
    {
        M.SetIdentity();
        MOw.SetIdentity();
        MTwwp.SetIdentity();
    }

}

} //namespace ORB_SLAM
