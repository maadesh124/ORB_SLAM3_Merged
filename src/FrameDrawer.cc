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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>


using namespace std;
namespace ORB_SLAM3
{

Object* ob;

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)

{


    initialized=false;

    mState=Tracking::SYSTEM_NOT_READY;

    mIm = cv::Mat(600,350,CV_8UC3, cv::Scalar(0,0,0));

    mImRight = cv::Mat(600,350,CV_8UC3, cv::Scalar(0,0,0));


    

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


pangolin::OpenGlMatrix FrameDrawer::GetCurrentViewMatrix()
{
    cv::Mat Tcw;
    mCurrentFrame.GetPose(Tcw);  // fills Tcw with 4x4 matrix

    pangolin::OpenGlMatrix M;
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            M.m[4*j + i] = Tcw.at<float>(i, j);

    return M;
}


pangolin::OpenGlMatrix  FrameDrawer::GetProjectionMatrix()
{

float fx = mCurrentFrame.fx;
float fy = mCurrentFrame.fy;
float cx = mCurrentFrame.cx;
float cy = mCurrentFrame.cy;
int w = Global::frameWindowWidth;  // Image width for monocular frame
int h = Global::frameWindowHeight; // Image height for monocular frame
//cout<<"frame w "<<w<<"h "<<h<<"f"<<fx<<" "<<fy<<" "<<cx<<" "<<cy<<""<<endl;
float near = 0.001;
float far = 1000.0;

pangolin::OpenGlMatrix P=pangolin::ProjectionMatrix(
     w,h,   // image size
     fx,fy,      // focal lengths
     cx,cy,      // principal point
     near,far  // near and far clipping planes
);

    return P;
}


void FrameDrawer::initializeWindow(int w,int h)
{
    std::cout<<w<<" win s"<<h<<std::endl;
    pangolin::CreateWindowAndBind(Global::frameWindow,w,h);
    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    s_cam = pangolin::OpenGlRenderState(
            pangolin::ProjectionMatrix(w, h, 500, 500, w/2, h/2, 0.1, 1000),
            pangolin::ModelViewLookAt(3, 3, 3, 0, 0, 0, pangolin::AxisY)
        );
    handler = new pangolin::Handler3D(s_cam);
    pangolin::View& d_cam= pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -w/(float)h)
                    .SetHandler(handler);



    imageTexture=pangolin::GlTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    // shaderF=pangolin::GlSlProgram();
    shaderF.AddShader(pangolin::GlSlVertexShader, vertex_shader);
    shaderF.AddShader(pangolin::GlSlFragmentShader, fragment_shader);
    shaderF.Link();




                             Eigen::Vector3f pos(0.0f, 0.0f, 0.0f);
                Eigen::Matrix3f ori=Eigen::Matrix3f::Identity();
                Eigen::Vector3f objP(0.0f, 0.0f, 0.0f);
                Eigen::Matrix3f objO=Eigen::Matrix3f::Identity();

             Anchor* te= mpAtlas->GetCurrentMap()->createAnchor(&pos,&ori);
             cout<<"mp id in creation"<<mpAtlas->GetCurrentMap()->GetId()<<endl;
              std::string path = "/home/samsung/Dev/Test/build/models/model.obj";
            ob = mpAtlas->GetCurrentMap()->createObject(&objP,&objO,te,path,0.005f);
    //pangolin::BindToContext(Global::mapWindow);
    initialized=true;

}

cv::Mat FrameDrawer::ConvertToRGB(const cv::Mat& im) {
    if(im.channels() == 3) {
        cv::Mat im_rgb;
        cv::cvtColor(im, im_rgb, cv::COLOR_BGR2RGB);
        return im_rgb;
    } else if(im.channels() == 1) {
        cv::Mat im_rgb;
        cv::cvtColor(im, im_rgb, cv::COLOR_GRAY2RGB);
        return im_rgb;
    }
    return im.clone();
}


void FrameDrawer::DrawObjects()
{

    // pangolin::glDrawAxis(2);
    // Map* curMap=mpAtlas->GetCurrentMap();
    // pangolin::OpenGlMatrix model;
    // model.SetIdentity();


    // glClear(GL_DEPTH_BUFFER_BIT);
    // int width=600,height=350;
    // pangolin::OpenGlMatrix proj=GetProjectionMatrix();
    //         model.m[0] *= ob->scaleFactor; // scale X
    //     model.m[5] *= ob->scaleFactor; // scale Y
    //     model.m[10] *= ob->scaleFactor; 
    // shaderF.Bind();


    //        shaderF.SetUniform("model", model);
    //     shaderF.SetUniform("view", GetCurrentViewMatrix());
    //     shaderF.SetUniform("projection",proj);
    //     ob->Draw();        

// cout<<"mp id in FrameDrawer"<<mpAtlas->GetCurrentMap()->GetId()<<endl;
//     cout<<"n obs Frame Drawer"<<curMap->objects.size()<<endl;

    // for(Object* object:curMap->objects)
    // {
    //     cout<<"ob present"<<endl;
    //     Eigen::Matrix3f globOri=object->anchor->ori*object->ori ;
    //     Eigen::Vector3f globPos=object->pos + object->anchor->pos;
    //     model=Global::ToOpenGlMatrix(model,globOri,globPos);

    //     model.m[0] *= object->scaleFactor; // scale X
    //     model.m[5] *= object->scaleFactor; // scale Y
    //     model.m[10] *= object->scaleFactor; // scale Z
    //     glClear(GL_DEPTH_BUFFER_BIT);
    //     pangolin::glDrawAxis(2);
    //     // glClear(GL_DEPTH_BUFFER_BIT);
    //     // shaderF.SetUniform("model", model);
    //     // shaderF.SetUniform("view", s_cam.GetModelViewMatrix());
    //     // shaderF.SetUniform("projection",s_cam.GetProjectionMatrix());
    //     // pangolin::glDrawColouredCube();
    //     // glClear(GL_DEPTH_BUFFER_BIT);

    //     // object->Draw();

    // }
   //  shaderF.Unbind();
}


void FrameDrawer::DrawImageAndObjects(float scale)
{
    
    cv::Mat im=DrawFrame(scale);//adds mapPoint visuals and text below image to current image

    if(im.empty()) return;

    if(!initialized)
    initializeWindow(Global::frameWindowWidth,Global::frameWindowHeight+20);


    pangolin::BindToContext(Global::frameWindow);

    //PrintOpenGlMatrix(s_cam.GetModelViewMatrix());
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
    imageTexture.RenderToViewportFlipY();

     glClear(GL_DEPTH_BUFFER_BIT);
     //pangolin::glDrawAxis(2);


    DrawObjects();
    pangolin::FinishFrame();
    pangolin::BindToContext(Global::mapWindow);
}



cv::Mat FrameDrawer::DrawFrame(bool bOldFeatures)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state; // Tracking state

    Frame currentFrame;
    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
            cv::line(im,(*it).first,(*it).second, cv::Scalar(0,255,0),5);

    }
    else if(state==Tracking::OK && bOldFeatures) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }

        }
    }
    else if(state==Tracking::OK && !bOldFeatures)
    {
        mnTracked=0;
        int nTracked2 = 0;
        mnTrackedVO=0;
        int n = vCurrentKeys.size();

        for(int i=0; i < n; ++i)
        {

            // This is a match to a MapPoint in the map
            if(vbMap[i])
            {
                mnTracked++;
            }
        }

        map<long unsigned int, cv::Point2f>::iterator it_match = mMatchedInImage.begin();
        while(it_match != mMatchedInImage.end())
        {
            long unsigned int mp_id = it_match->first;
            cv::Point2f p_image = it_match->second;

            if(mProjectPoints.find(mp_id) != mProjectPoints.end())
            {
                cv::Point2f p_proj = mMatchedInImage[mp_id];
                cv::line(im, p_proj, p_image, cv::Scalar(0, 255, 0), 2);
                nTracked2++;
            }
            else
            {
                cv::circle(im,p_image,2,cv::Scalar(0,0,255),-1);
            }


            it_match++;
        }

        n = vOutlierKeys.size();
        for(int i=0; i < n; ++i)
        {
            cv::Point2f point3d_proy;
            float u, v;
            currentFrame.ProjectPointDistort(vpOutlierMPs[i] , point3d_proy, u, v);

            cv::Point2f point_im = vOutlierKeys[i].pt;

            cv::line(im,cv::Point2f(u, v), point_im,cv::Scalar(0, 0, 255), 1);
        }

    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                         cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            if(vbVO[i + Nleft] || vbMap[i + Nleft])
            {
                cv::Point2f pt1,pt2;
                pt1.x=mvCurrentKeysRight[i].pt.x-r;
                pt1.y=mvCurrentKeysRight[i].pt.y-r;
                pt2.x=mvCurrentKeysRight[i].pt.x+r;
                pt2.y=mvCurrentKeysRight[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i + Nleft])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,mvCurrentKeysRight[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,mvCurrentKeysRight[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}



void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;

    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    //Variables for the new visualization
    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;

                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }

    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

FrameDrawer::~FrameDrawer()
{
delete handler;
handler = nullptr;
    std::cout << "FrameDrawer destroyed" << std::endl;
}

} //namespace ORB_SLAM
