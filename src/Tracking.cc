/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"
#include "TIMER.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"
#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>
#include <mutex>
#include <sstream>
#include <include/Fundamental.h>

using namespace std;
namespace ORB_SLAM2 {

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if (sensor==System::STEREO || sensor==System::RGBD) {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if (sensor==System::RGBD) {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}

cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
                                  const double &timestamp, const vector<std::pair<vector<double>, int>>& bounding_box)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft,
                          mpORBextractorRight, mpORBVocabulary, mK, mDistCoef,
                          mbf, mThDepth, bounding_box);

    cout << "ID:" << mCurrentFrame.mnId << endl;
    Track();

    cout << "当前帧物体：" << mCurrentFrame.objects_cur_.size() << endl;
    for (int i = 0; i < mCurrentFrame.objects_cur_.size(); ++i) {
         if (mCurrentFrame.objects_cur_[i]->mnId_ != -1)
             cout << mCurrentFrame.objects_cur_[i]->mnId_ << endl;
    }

    cout << "地图中物体：" << mpMap->objects_in_map_.size() << endl;
    for (int j = 0; j < mpMap->objects_in_map_.size(); ++j) {
        if (mpMap->objects_in_map_[j])
            cout << mpMap->objects_in_map_[j]->mnId_ << endl;
    }
    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty()) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }
        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK) {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame()) {
                CreateNewKeyFrame();
            }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState==LOST) {
            if (mpMap->KeyFramesInMap()<=5) {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }
}

void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);
        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame.N;i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0 ) {//&& !mCurrentFrame.IsInDynamicBox(i)
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);
                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
        }
        //! create object keypoints
        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);
        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
        mpMap->mvpKeyFrameOrigins.push_back(pKFini);
        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    if(nmatches < 15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if (mCurrentFrame.mvbOutlier[i] ) {//|| mCurrentFrame.IsInDynamicBox(i)
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

string Convert(float Num) {
    ostringstream oss;
    oss << Num;
    string str(oss.str());
    return str;
}

void Tracking::JudgeDynamicObject(){
    cv::Mat neg_KT = mK.t().inv();
    cv::Mat neg_K = mK.inv();
    cv::Mat T21 = mCurrentFrame.mTcw * mLastFrame.mTcw.inv();
    cv::Mat t21 = T21.colRange(3, 4).rowRange(0, 3).clone();
    cv::Mat R21 = T21.colRange(0, 3).rowRange(0, 3).clone();
    cv::Mat E = mCurrentFrame.skew(t21) * R21;
    cv::Mat FF = neg_KT * E * neg_K;

    cv::Mat image_cur;
    image_cur = mCurrentFrame.current_frame_image.clone();
    cv::cvtColor(image_cur, image_cur,  cv::COLOR_GRAY2RGB);
    cv::Mat image_last = mLastFrame.current_frame_image.clone();
    cv::cvtColor(image_last, image_last,  cv::COLOR_GRAY2RGB);

    //! 画光流跟踪的点
    for (int j = 0; j < mCurrentFrame.optical_flow_points_pairs_filter_.size(); ++j) {
        cv::Point2f pre_pt = mCurrentFrame.optical_flow_points_pairs_filter_[j].first;
        cv::Point2f cur_pt = mCurrentFrame.optical_flow_points_pairs_filter_[j].second;
        cv::circle(image_cur, cur_pt, 3, cv::Scalar(0, 0, 255));
        cv::line(image_cur, pre_pt, cur_pt, cv::Scalar(0, 255, 0));
    }

    //! 框外点
    int object_cur = mCurrentFrame.objects_cur_.size();
    vector<cv::KeyPoint> last_frame_points;
    vector<cv::KeyPoint> cur_frame_points;
    vector<cv::KeyPoint> last_frame_points_out_box;
    vector<cv::KeyPoint> cur_frame_points_out_box;
    vector<cv::Point2f> last_frame_points_out_box_for_draw;
    for (auto it = mCurrentFrame.matches_out_box.begin();  it != mCurrentFrame.matches_out_box.end(); ++it) {
        last_frame_points.push_back(mLastFrame.mvKeysUn[it->second]);
        cur_frame_points.push_back(mCurrentFrame.mvKeysUn[it->first]);
        last_frame_points_out_box.push_back(mLastFrame.mvKeysUn[it->second]);
        last_frame_points_out_box_for_draw.push_back(mLastFrame.mvKeysUn[it->second].pt);
        cur_frame_points_out_box.push_back(mCurrentFrame.mvKeysUn[it->first]);
    }

    //! 框内点
    vector<cv::KeyPoint> last_frame_points_in_box;
    vector<cv::KeyPoint> cur_frame_points_in_box;
    vector<cv::Point2f> last_frame_points_in_box_for_draw;
    vector<int> point_in_box_id;
    vector<int> points_in_box_filtered(object_cur, 0);
    for (auto it = mCurrentFrame.matches_in_box.begin(); it != mCurrentFrame.matches_in_box.end(); ++it) {
        last_frame_points.push_back(mLastFrame.mvKeysUn[it->second.first]);
        cur_frame_points.push_back(mCurrentFrame.mvKeysUn[it->first]);
        if (mCurrentFrame.mappoint_mapping_to_object_.count(it->first)) {
            last_frame_points_in_box.push_back(mLastFrame.mvKeysUn[it->second.first]);
            last_frame_points_in_box_for_draw.push_back(mLastFrame.mvKeysUn[it->second.first].pt);
            cur_frame_points_in_box.push_back(mCurrentFrame.mvKeysUn[it->first]);
            point_in_box_id.push_back(it->second.second);
            points_in_box_filtered[it->second.second]++;
        } else {
            last_frame_points_out_box.push_back(mLastFrame.mvKeysUn[it->second.first]);
            last_frame_points_out_box_for_draw.push_back(mLastFrame.mvKeysUn[it->second.first].pt);
            cur_frame_points_out_box.push_back(mCurrentFrame.mvKeysUn[it->first]);
        }
    }

    //! 计算基础矩阵，验证对极约束
    vector<bool> vbMatchesInliers, vbMatchesInliers_in_box;
    cv::Mat F21;
    vector<int> dynamic_points_in_box;
    dynamic_points_in_box.resize(object_cur);
    vector<bool> is_dynamic_box(object_cur, false);
    std::vector<cv::Vec<float, 3>> epilines2;
    int outlier_tmp = 0;
    if (last_frame_points.size() > 8) {
        Cal_Fundamental(last_frame_points, cur_frame_points, vbMatchesInliers, F21);
        if (last_frame_points_in_box.size() != 0) {
            Check_Fundamental(last_frame_points_in_box, cur_frame_points_in_box, FF, vbMatchesInliers_in_box);
            cv::computeCorrespondEpilines(last_frame_points_in_box_for_draw, 1, FF, epilines2);
            for (int k = 0; k < vbMatchesInliers_in_box.size(); ++k) {
                if(!vbMatchesInliers_in_box[k]) {
                    outlier_tmp++;
                    int dynamic_box_id = point_in_box_id[k];
                    dynamic_points_in_box[dynamic_box_id]++;
                }
            }
        }
    }

    //! 判断是否是动态物体
    for (int l = 0; l < object_cur; ++l) {
        float points_num = points_in_box_filtered[l];
        float dynamic_points_num = dynamic_points_in_box[l];
        float dynamic_ratio = dynamic_points_num / points_num;
        if (points_num <= 5 && dynamic_ratio > 0.8)
            is_dynamic_box[l] = true;
        if (points_num > 5 && dynamic_ratio > 0.3) {
            is_dynamic_box[l] = true;
        }
    }

    cv::Mat outImg(image_cur.rows + image_last.rows + 1,
                   image_cur.cols, image_cur.type());
    mCurrentFrame.DrawBox(is_dynamic_box, image_cur);
    int h = image_last.rows;
    int w = image_last.cols;

    image_last.copyTo(outImg.rowRange(0, h));
    image_cur.copyTo(outImg.rowRange(h + 1, outImg.rows));

    //! 画特征点匹配线和极线
    cv::Scalar point_and_line_color;
    /* for (int kk = 0; kk < cur_frame_points_out_box.size(); ++kk) {
             if (vbMatchesInliers[kk]) {
                 point_and_line_color = cv::Scalar(0, 255, 0);
             } else {
                 point_and_line_color = cv::Scalar(0, 0, 255);
             }
            cv::circle(outImg, last_frame_points_out_box[kk].pt, 3, point_and_line_color);
            cv::circle(outImg, cv::Point2f(cur_frame_points_out_box[kk].pt.x, image_cur.rows + cur_frame_points_out_box[kk].pt.y), 3, point_and_line_color);
            cv::line(outImg, last_frame_points_out_box[kk].pt, cv::Point2f(cur_frame_points_out_box[kk].pt.x,
                                                                           image_cur.rows + cur_frame_points_out_box[kk].pt.y), point_and_line_color);
        }*/

    int inlier_tmp = 0;
    for (int m = 0; m < vbMatchesInliers_in_box.size(); ++m) {
        if (vbMatchesInliers_in_box[m]) {
            inlier_tmp++;
            point_and_line_color = cv::Scalar(0, 255, 0);
        } else {
            point_and_line_color = cv::Scalar(0, 0, 255);
        }
        cv::circle(outImg, last_frame_points_in_box[m].pt, 3, point_and_line_color, -1);
        cv::circle(outImg, cv::Point2f(cur_frame_points_in_box[m].pt.x, image_cur.rows + cur_frame_points_in_box[m].pt.y), 3, point_and_line_color, -1);
//        cv::line(outImg, last_frame_points_in_box[m].pt, cv::Point2f(cur_frame_points_in_box[m].pt.x,
//                                                                     image_cur.rows + cur_frame_points_in_box[m].pt.y), point_and_line_color);
    }

    //! 画物体的中心点在图像上的位置
    for (int i = 0; i < mCurrentFrame.objects_cur_.size(); ++i) {
        cv::Mat pos_world = mCurrentFrame.objects_cur_[i]->Pos_;
        if (pos_world.empty())
            continue;
        cv::Mat pos_camera = mCurrentFrame.GetMRw() * pos_world + mCurrentFrame.GetMtw();
        float depth_val = pos_camera.at<float>(2);
        cv::Mat pos_pixel = mK * pos_camera;
        float u_val = pos_pixel.at<float>(0) / depth_val;
        float v_val = pos_pixel.at<float>(1) / depth_val;
        cv::circle(outImg, cv::Point2f(u_val, image_cur.rows + v_val), 5, cv::Scalar(0, 200, 255), -1);
    }
    cv::imshow("current frame", image_cur);
    cv::waitKey(1e3 / 30);//1e3 / 30
}

bool CropImage(const int& width, const int& height, cv::Rect2d& box) {
    if (box.x < 0) {
        box.width += box.x;
        box.x = 0;
    } 
    if (box.x + box.width > width) {
        box.width = width - box.x;
    }
    if (box.y < 0) {
        box.height += box.y;
        box.y = 0;
    } 
    if (box.y + box.height > height) {
        box.height = height - box.y;
    }
    return (box.x >= 0 && box.width > 0 && box.y >= 0 && box.height > 0
    && box.x + box.width <= width && box.y + box.height <= height);
}

bool CropImageAndTemplate(const int& width, const int& height, cv::Rect2d& box, cv::Rect2d& box_template) {
        if (box.x < 0) {
            box.width += box.x;
            box_template.x = (box_template.x + box.x > 0) ? box_template.x + box.x : 0;
            box_template.width += box.x;
            box.x = 0;
        }
        if (box.x + box.width > width) {
            box.width = width - box.x;
            box_template.width = width - box.x;
        }
        if (box.y < 0) {
            box.height += box.y;
            box_template.y = (box_template.y + box.y > 0) ? box_template.y + box.y : 0;
            box_template.height += box.y;
            box.y = 0;
        }
        if (box.y + box.height > height) {
            box.height = height - box.y;
            box_template.height = height - box.y;
        }
        bool box_valid = (box.x >= 0 && box.width > 0 && box.y >= 0 && box.height > 0
                          && box.x + box.width <= width && box.y + box.height <= height);
        bool box_template_valid = (box_template.x >= 0 && box_template.width > 0 && box_template.y >= 0 && box_template.height > 0
                             && box_template.x + box_template.width <= width && box_template.y + box_template.height <= height);
        return (box_valid && box_template_valid);
    }

void Tracking::MultiScaleTemplateMatch(const vector<vector<double>>& tracking_object_box,
        const vector<int>& tracking_object_box_class) {
    cv::Mat image_last = mLastFrame.current_frame_image.clone();
    cv::Mat image_cur = mCurrentFrame.current_frame_image.clone();
    cv::cvtColor(image_last, image_last,  cv::COLOR_GRAY2RGB);
    cv::cvtColor(image_cur, image_cur,  cv::COLOR_GRAY2RGB);
    for (int i = 0; i < tracking_object_box.size(); ++i) {
        if (!mCurrentFrame.valid_tracker_[i])
            continue;
        vector<double> box_last = mLastFrame.tracking_object_box_[i];
        cv::Rect2d rect_template(box_last[0], box_last[2], box_last[1] - box_last[0], box_last[3] - box_last[2]);
        if (!CropImage(mLastFrame.current_frame_image.cols, mLastFrame.current_frame_image.rows, rect_template))
            continue;
        double left = tracking_object_box[i][0];
        double right = tracking_object_box[i][1];
        double top = tracking_object_box[i][2];
        double bottom = tracking_object_box[i][3];
        double width = right - left;
        double height = bottom - top;
        cv::Rect2d rect_predict(left, top, width, height);
        /*
        if (mCurrentFrame.mnId > 110) {
            if (rect_predict.x > 0 && rect_predict.y > 0 &&
                rect_predict.x + rect_predict.width < mLastFrame.current_frame_image.cols &&
                rect_predict.y + rect_predict.height < mLastFrame.current_frame_image.rows) {
                cv::Mat image_predict1 = image_cur(rect_predict);
                cv::imshow("predict1", image_predict1);
            }
            if (rect_template.x > 0 && rect_template.y > 0 &&
                rect_template.x + rect_template.width < mLastFrame.current_frame_image.cols &&
                rect_template.y + rect_template.height < mLastFrame.current_frame_image.rows) {
                cv::Mat image_template1 = image_last(rect_template);
                cv::imshow("temp1", image_template1);
            }
        }*/

        //! 如果预测框超出图像边界，同等裁剪模板
        if (!CropImageAndTemplate(mLastFrame.current_frame_image.cols, mLastFrame.current_frame_image.rows, rect_predict, rect_template))
            continue;
        double margin_x = 0.3 * width;
        double margin_y = 0.3 * height;
        cv::Rect2d rect_search(left - margin_x, top - margin_y, width + 2 * margin_x, height + 2 * margin_y);
        //! 如果扩展区域超出图像边界
        if (!CropImage(mLastFrame.current_frame_image.cols, mLastFrame.current_frame_image.rows, rect_search))
            continue;
        cv::Mat image_template = image_last(rect_template);
        cv::Mat image_search = image_cur(rect_search);

        double scale = 0.8;
        double maxScore = 0;
        double bestScale;
        cv::Point bestVal;
        for (int j = 0; j < 10; ++j) {
            cv::Mat image_search_scale;
            cv::Size s(image_search.cols * scale, image_search.rows * scale);
            cv::resize(image_search, image_search_scale, s);
            cv::Mat dstImg;
            if (image_search_scale.rows < image_template.rows || image_search_scale.cols < image_template.cols)
                continue;
            cout << image_search_scale.rows << " " << image_template.rows << " " << image_search_scale.cols << " " << image_template.cols << endl;
            cv::matchTemplate(image_search_scale, image_template, dstImg, 3);
            cv::Point minPoint, maxPoint;
            double minVal = 0;
            double maxVal = 0;
            cv::minMaxLoc(dstImg, &minVal, &maxVal, &minPoint, &maxPoint);
            if (maxVal > maxScore) {
                maxScore = maxVal;
                bestScale = scale;
                bestVal = maxPoint;
            }
            scale += 0.05;
        }

        if (maxScore < 0.87)
            continue;
        /*
        if (mCurrentFrame.mnId > 110) {
            cv::imshow("temp", image_template);
            cv::imshow("search", image_search);
            cv::waitKey(0);
        }*/

        //! 在缩放后的图像上显示box
        cv::Point2f box_left_top(bestVal.x / bestScale, bestVal.y / bestScale);
        double width_scale = image_template.cols / bestScale;
        double height_scale = image_template.rows / bestScale;
        if (width_scale < 30 || height_scale < 30)
            continue;
        vector<double> box;
        box.push_back(rect_search.x + box_left_top.x);
        box.push_back(rect_search.x + box_left_top.x + width_scale);
        box.push_back(rect_search.y + box_left_top.y);
        box.push_back(rect_search.y + box_left_top.y + height_scale);
        //! 保存上一次的运动状态
        double center_x_last = (box_last[1] + box_last[0]) / 2.;
        double center_y_last = (box_last[3] + box_last[2]) / 2.;
        double center_x_cur = rect_search.x + box_left_top.x + width_scale / 2.;
        double center_y_cur = rect_search.y + box_left_top.y + height_scale / 2.;
        cv::Point2f motion(center_x_cur - center_x_last, center_y_cur - center_y_last);
        mCurrentFrame.tracking_object_box_.push_back(box);
        mCurrentFrame.tracking_object_box_class_.push_back(tracking_object_box_class[i]);
        mCurrentFrame.tracking_object_motion_.push_back(motion);
        //! 包括框大小通过验证的光流跟踪的点
        for (int k = 0; k < mCurrentFrame.optical_flow_points_pairs_[i].size(); ++k) {
            mCurrentFrame.optical_flow_points_pairs_filter_.push_back(mCurrentFrame.optical_flow_points_pairs_[i][k]);
        }
    }
}

struct MapPointDepthCompare {
    bool operator() (const std::pair<float, int>& v1, const std::pair<float, int>& v2) const {
        return v1.first < v2.first;
    }
};

void Tracking::RemovePointsBackground() {
    size_t box_size = mCurrentFrame.tracking_object_box_.size();
    vector<vector<int>> points_in_track_box;
    vector<int> points_in_track_box_class;
    points_in_track_box.resize(box_size);
    points_in_track_box_class.resize(box_size);
    for (size_t i = 0u; i < mCurrentFrame.N; ++i) {
        int box_id, class_id;
        if (mCurrentFrame.IsInTrackBox(i, box_id, class_id)) {
            points_in_track_box[box_id].push_back(i);
            points_in_track_box_class[box_id] = class_id;
        }
    }

    for (size_t l = 0u; l < mCurrentFrame.tracking_object_box_.size(); ++l) {
        vector<double> bbox = mCurrentFrame.tracking_object_box_[l];
        double box_center_x = 0.5 * (bbox[1] + bbox[0]);
        double box_center_y = 0.5 * (bbox[3] + bbox[2]);
        cv::Point2f center_point(box_center_x, box_center_y);
        multiset<std::pair<float, int>, MapPointDepthCompare> mappoints;
        for (int j = 0; j < points_in_track_box[l].size(); ++j) {
            int keypoint_index = points_in_track_box[l][j];
            MapPoint* mp = mCurrentFrame.mvpMapPoints[keypoint_index];
            if (mp) {
                cv::Mat pos = mp->GetWorldPos();
                mappoints.emplace(pos.at<float>(2), keypoint_index);
            }
        }
        if (mappoints.size() < 2)
            continue;

        int count = 0;
        int nsize = mappoints.size();
        int start_index = nsize / 4;
        int end_index = 3 * nsize / 4;
        if (nsize & 1 == 1)//! 奇数停止位置向前一个
            end_index -= 1;

        //! 寻找深度处于中间的点集合
        vector<int> center_index;
        for (auto iter = mappoints.begin();  iter != mappoints.end(); ++iter) {
            if (count >= start_index && count <= end_index) {
                center_index.push_back(iter->second);
            } else if (count > end_index) {
                break;
            }
            count++;
        }
        if (center_index.size() == 0)
            continue;
        //! 计算深度处于中间的点距离box中心最近的点
        float min_distance = 9999.;
        int center_point_index = -1;
        for (int k = 0; k < center_index.size(); ++k) {
            int keypoint_index = center_index[k];
            cv::Point2f kp = mCurrentFrame.mvKeysUn[keypoint_index].pt;
            float distance = norm(kp - center_point);
            if (distance < min_distance) {
                min_distance = distance;
                center_point_index = keypoint_index;
            }
        }
        cv::Mat centroid = mCurrentFrame.mvpMapPoints[center_point_index]->GetWorldPos();
        //! 计算box内的每个点距离中心点的深度距离，只选择距近的点
        for (int j = 0; j < points_in_track_box[l].size(); ++j) {
            int keypoint_index = points_in_track_box[l][j];
            MapPoint* mp = mCurrentFrame.mvpMapPoints[keypoint_index];
            if (mp) {
                cv::Mat pos = mp->GetWorldPos();
                float distance_each_mappoint = fabs(pos.at<float>(2) - centroid.at<float>(2));
                if (distance_each_mappoint > 5.)
                    continue;
                mCurrentFrame.points_for_optical_flow.emplace(keypoint_index, make_pair(l, points_in_track_box_class[l]));
            }
        }
    }
}

void Tracking::UseOpticalFlowTrack() {
    if (mCurrentFrame.mnId == 2 || mCurrentFrame.mnId % 10 == 0) {
        mCurrentFrame.read_detect_state_ = true;
        for (int i = 0; i < mCurrentFrame.objects_cur_detect_.size(); ++i) {
            vector<double> box = mCurrentFrame.objects_cur_detect_[i]->bounding_box_;
            mCurrentFrame.tracking_object_box_.push_back(box);
            mCurrentFrame.tracking_object_box_class_.push_back(mCurrentFrame.objects_cur_detect_[i]->class_id_);
        }
        for (int i = 0; i < mCurrentFrame.N; ++i) {
            int box_id, class_id;
            if (mCurrentFrame.IsInTrackBox(i, box_id, class_id)) {
                mCurrentFrame.points_for_optical_flow.emplace(i, make_pair(box_id, class_id));
            }
        }
        LK_tracker.Init(mCurrentFrame, mCurrentFrame.points_for_optical_flow);
    } else {
        LK_tracker.CurrentFrame_ = mCurrentFrame;
        LK_tracker.TrackImage();
        mCurrentFrame.optical_flow_points_pairs_ = LK_tracker.optical_flow_points_pairs_;
        mCurrentFrame.valid_tracker_ = LK_tracker.valid_tracker_;
        int box_size = mLastFrame.tracking_object_box_.size();
        vector<vector<double>> tracking_object_box;
        tracking_object_box.resize(box_size);
        vector<int> tracking_object_box_class;
        tracking_object_box_class.resize(box_size);
        for (int j = 0; j < box_size; ++j) {
            if (!mCurrentFrame.valid_tracker_[j])
                continue;
            vector<double> box = mLastFrame.tracking_object_box_[j];
            double x_last = (box[1] + box[0]) / 2.;
            double y_last = (box[3] + box[2]) / 2.;
            double x_cur = x_last + LK_tracker.box_center_motion[j].x;
            double y_cur = y_last + LK_tracker.box_center_motion[j].y;
            mCurrentFrame.predict_box_center_vec.push_back(cv::Point2f(x_cur, y_cur));
            double width = box[1] - box[0];
            double height = box[3] - box[2];
            vector<double> box_;
            double left = x_cur - width / 2.;
            double right = x_cur + width / 2.;
            double top = y_cur - height / 2.;
            double bottom = y_cur + height / 2.;
            box_.push_back(left);
            box_.push_back(right);
            box_.push_back(top);
            box_.push_back(bottom);
            tracking_object_box[j] = box_;
            tracking_object_box_class[j] = LK_tracker.box_class_[j];
        }
        MultiScaleTemplateMatch(tracking_object_box, tracking_object_box_class);
        //! 深度去除背景点进行光流跟踪，效果不太好
        // RemovePointsBackground();

        //! 用所有点进行光流跟踪
        for (int i = 0; i < mCurrentFrame.N; ++i) {
            int box_id, class_id;
            if (mCurrentFrame.IsInTrackBox(i, box_id, class_id)) {
                mCurrentFrame.points_for_optical_flow.emplace(i, make_pair(box_id, class_id));
            }
        }
        LK_tracker.Init(mCurrentFrame, mCurrentFrame.points_for_optical_flow);
    }

    for (int k = 0; k < mCurrentFrame.tracking_object_box_.size(); ++k) {
        std::shared_ptr<Object> obj = make_shared<Object>();
        obj->bounding_box_ = mCurrentFrame.tracking_object_box_[k];
        obj->class_id_ = mCurrentFrame.tracking_object_box_class_[k];
        mCurrentFrame.objects_cur_.push_back(obj);
    }
}

bool Tracking::TrackWithMotionModel() {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    //! 光流跟踪
    //! 如果这个函数放在SearchByProjection之前，就不可以使用深度聚类区分背景点和物体点
    UseOpticalFlowTrack();
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches<20) {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2*th, mSensor==System::MONOCULAR);
    }
    if(nmatches<20)
        return false;

    mCurrentFrame.GetFrameObject(mpMap);
    Optimizer::PoseOptimization(&mCurrentFrame);
    JudgeDynamicObject();

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {//|| mCurrentFrame.IsInDynamicBox(i)
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }
    if (mCurrentFrame.mnId >= 2 && !mbOnlyTracking) {
        for (int i = 0; i < mpMap->objects_in_map_.size() ; ++i) {
            if (mpMap->objects_in_map_[i] ==NULL)
                continue;
            if (mpMap->objects_in_map_[i]->dynamic_) {
                mpMap->objects_in_map_[i].reset();// 计算物体个数的时候要判断NULL
                cout << "Delete dynamic object!" << endl;
            }
            if (mpMap->objects_in_map_[i]) {
                if (mpMap->objects_in_map_[i]->add_frame_id == mCurrentFrame.mnId - 2) {
                    if (mpMap->objects_in_map_[i]->observe_times_ < 2) {
                        mpMap->objects_in_map_[i].reset();// 计算物体个数的时候要判断NULL
                        cout << "Delete observe few times object!" << endl;
                    }
                }
            }
        }
    }
//    mpMap->CheckFuseMapObject();
    if (mbOnlyTracking) {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }
    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap() {
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.
    UpdateLocalMap();
    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor == System::STEREO )//|| mCurrentFrame.IsInDynamicBox(i)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}

bool Tracking::NeedNewKeyFrame() {
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

} //namespace ORB_SLAM
