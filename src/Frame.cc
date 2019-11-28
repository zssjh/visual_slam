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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>
#include <string.h>

namespace ORB_SLAM2 {
    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations=true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame()
    {}

//Copy Constructor
    Frame::Frame(const Frame &frame)
            :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
             mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
             mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
             mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
             mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
             mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
             mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
             mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
             mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
             mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
             mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
             objects_cur_(frame.objects_cur_), tracking_object_box_(frame.tracking_object_box_),
             mappoint_mapping_to_object_(frame.mappoint_mapping_to_object_ ), current_frame_image(frame.current_frame_image),
             current_frame_image_log(frame.current_frame_image_log), read_detect_state_(frame.read_detect_state_){
        for(int i=0; i<FRAME_GRID_COLS; i++)
            for(int j=0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j];
        if(!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
                 ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc,
                 cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
                 const vector<std::pair<vector<double>, unsigned int>>& bounding_box)
            : mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight),
              mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
              mpReferenceKF(static_cast<KeyFrame*>(NULL)), current_frame_image(imLeft), read_detect_state_(false) {
        mnId=nNextId++;
        // remove similar box
        for (int i = 0; i < bounding_box.size(); ++i) {
            bool single = true;
            for (int j = i + 1; j < bounding_box.size(); ++j) {
                int left_dis = abs(bounding_box [i].first[0] - bounding_box [j].first[0]);
                int right_dis = abs(bounding_box [i].first[1] - bounding_box [j].first[1]);
                int top_dis = abs(bounding_box [i].first[2] - bounding_box [j].first[2]);
                int bottom_dis = abs(bounding_box [i].first[3] - bounding_box [j].first[3]);
                if (left_dis < 35 && right_dis < 35 && top_dis < 35 && bottom_dis < 35) {
                    single = false;
                    break;
                }
            }
            if (single) {
                std::shared_ptr<Object> obj = make_shared<Object>();
                obj->bounding_box_ = bounding_box[i].first;
                obj->class_id_ = bounding_box[i].second;
                objects_cur_.push_back(obj);
            }
        }

        //! 用于模板匹配的图像的对数转换：current_frame_image_log，效果不好
        current_frame_image_log = cv::Mat::zeros(imLeft.size(), imLeft.type());
        double gray;
        double c = 1.;
        for (int i = 0; i < imLeft.rows; i++) {
            for (int j = 0; j < imLeft.cols; j++) {
                gray = (double)imLeft.at<uchar>(i, j);
                gray = c * log((double)(1 + gray));
                current_frame_image_log.at<uchar>(i, j) = cv::saturate_cast<uchar>(gray);
            }
        }
        cv::normalize(current_frame_image_log, current_frame_image_log, 0, 255, cv::NORM_MINMAX);
        cv::convertScaleAbs(current_frame_image_log, current_frame_image_log);

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
        thread threadRight(&Frame::ExtractORB,this,1,imRight);
        threadLeft.join();
        threadRight.join();

        N = mvKeys.size();
        if(mvKeys.empty())
            return;

        UndistortKeyPoints();
        ComputeStereoMatches();
        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations) {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }
        mb = mbf/fx;
        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth) {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0,imGray);

        N = mvKeys.size();

        if(mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }
        mb = mbf/fx;
        AssignFeaturesToGrid();
    }


    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
             mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth) {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0,imGray);

        N = mvKeys.size();

        if(mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);

        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }

        mb = mbf/fx;

        AssignFeaturesToGrid();
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);

        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if(flag==0)
            (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
        else
            (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);

        if(dist<minDistance || dist>maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn)/dist;

        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf*invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;
                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }
        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        return true;
    }

    void Frame::ComputeBoW()
    {
        if(mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }
        // Fill matrix with points
        cv::Mat mat(N,2,CV_32F);
        for(int i=0; i<N; i++)
        {
            mat.at<float>(i,0)=mvKeys[i].pt.x;
            mat.at<float>(i,1)=mvKeys[i].pt.y;
        }
        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);
        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if(mDistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
            mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
            mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
            mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

            // Undistort corners
            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
            mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
            mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
            mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N,-1.0f);
        mvDepth = vector<float>(N,-1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        //Assign keypoints to row table
        vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

        for(int i=0; i<nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for(int iR=0; iR<Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY+r);
            const int minr = floor(kpY-r);

            for(int yi=minr;yi<=maxr;yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf/minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(N);

        for(int iL=0; iL<N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if(vCandidates.empty())
                continue;

            const float minU = uL-maxD;
            const float maxU = uL-minD;

            if(maxU<0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for(size_t iC=0; iC<vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                    continue;

                const float &uR = kpR.pt.x;

                if(uR>=minU && uR<=maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                    if(dist<bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if(bestDist<thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x*scaleFactor);
                const float scaledvL = round(kpL.pt.y*scaleFactor);
                const float scaleduR0 = round(uR0*scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
                IL.convertTo(IL,CV_32F);
                IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2*L+1);

                const float iniu = scaleduR0+L-w;
                const float endu = scaleduR0+L+w+1;
                if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for(int incR=-L; incR<=+L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                    IR.convertTo(IR,CV_32F);
                    IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                    float dist = cv::norm(IL,IR,cv::NORM_L1);
                    if(dist<bestDist)
                    {
                        bestDist =  dist;
                        bestincR = incR;
                    }

                    vDists[L+incR] = dist;
                }

                if(bestincR==-L || bestincR==L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L+bestincR-1];
                const float dist2 = vDists[L+bestincR];
                const float dist3 = vDists[L+bestincR+1];

                const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

                if(deltaR<-1 || deltaR>1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

                float disparity = (uL-bestuR);

                if(disparity>=minD && disparity<maxD)
                {
                    if(disparity<=0)
                    {
                        disparity=0.01;
                        bestuR = uL-0.01;
                    }
                    mvDepth[iL]=mbf/disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int,int>(bestDist,iL));
                }
            }
        }
        sort(vDistIdx.begin(),vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size()/2].first;
        const float thDist = 1.5f*1.4f*median;

        for (int i=vDistIdx.size()-1;i>=0;i--) {
            if(vDistIdx[i].first<thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second]=-1;
                mvDepth[vDistIdx[i].second]=-1;
            }
        }
    }

    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);
        for(int i=0; i<N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];
            const float &v = kp.pt.y;
            const float &u = kp.pt.x;
            const float d = imDepth.at<float>(v,u);
            if(d>0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x-mbf/d;
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i) {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            const float x = (u-cx)*z*invfx;
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
            return mRwc*x3Dc+mOw;
        }
        else
            return cv::Mat();
    }

    void Frame::ComputeCentroid(const vector<unsigned int>& all_mappoints,
                                cv::Mat& centroid,
                                map<size_t, double>& mappoints_distance_to_center_map) {
        int mappoints_size = all_mappoints.size();
        if (mappoints_size < 5) {
            return;
        }
        vector<size_t> AllIndices;
        for (int i = 0; i < mappoints_size; ++i) {
            AllIndices.push_back(i);
        }
        vector<vector<size_t>> minSets = vector<vector<size_t>>(50, vector<size_t>(5,0));
        vector<size_t> AvailableIndices;
        DUtils::Random::SeedRandOnce(0);
        double minDistance = 999999.;
        for (int it = 0; it < 50; it++) {
            double curDistance = 0;
            AvailableIndices = AllIndices;
            cv::Mat center_point_pos_sum(3, 1, CV_32F);
            cv::Mat center_point_pos_avg;
            for (size_t j = 0; j < 5; j++) {
                int randi = DUtils::Random::RandomInt(0, mappoints_size - 1);
                int idx = AvailableIndices[randi];
                minSets[it][j] = idx;
                cv::add(center_point_pos_sum, mvpMapPoints[all_mappoints[idx]]->GetWorldPos(), center_point_pos_sum);
                AvailableIndices[randi] = AvailableIndices.back();
                AvailableIndices.pop_back();
            }

            float pos_x = center_point_pos_sum.at<float>(0) / 5.;
            float pos_y = center_point_pos_sum.at<float>(1) / 5.;
            float pos_z = center_point_pos_sum.at<float>(2) / 5.;
            center_point_pos_avg = cv::Mat_<float>(3, 1) << pos_x, pos_y, pos_z;
            cv::Mat distance_pos(3, 1, CV_32F);

            for (int i = 0; i < mappoints_size; ++i) {
                cv::absdiff(mvpMapPoints[all_mappoints[i]]->GetWorldPos(), center_point_pos_avg, distance_pos);
                double distance_each_mappoint = norm(distance_pos);
                mappoints_distance_to_center_map.emplace(all_mappoints[i], distance_each_mappoint);
                curDistance += distance_each_mappoint;
            }
            if (curDistance < minDistance) {
                centroid = center_point_pos_avg;
                minDistance = curDistance;
            }
        }
    }

    //! 根据box中心点附近一定区域内的点的平均值计算物体的位置，多数是正确的，但是少数计算的位置是偏的
    void Frame::ComputeCentroid2D(const int& i, cv::Mat& centroid,
                                  map<size_t, double>& mappoints_distance_to_center_map) {
        int points_compute_center_num = 0;
        vector<double> bbox = objects_cur_[i]->bounding_box_;
        double box_width = bbox[1] - bbox[0];
        double box_height = bbox[3] - bbox[2];
        bool compute_center = true;
        int step = 5;
        while (compute_center && step > 1) {
            step--;
            double center_margin_width = box_width / step;
            double center_margin_height = box_height /step;
            cv::Point2f box_center((bbox[0] + bbox[1]) / 2., (bbox[2] + bbox[3])/ 2.);
            for (int j = 0; j < points_in_box[i].size(); ++j) {
                int keypoint_index = points_in_box[i][j];
                cv::Point2f keypoint = mvKeysUn[keypoint_index].pt;
                double diff_x = fabs(box_center.x - keypoint.x);
                double diff_y = fabs(box_center.y - keypoint.y);
                if (fabs(diff_x) < center_margin_width && fabs(diff_y) < center_margin_height) {
                    MapPoint* mp = mvpMapPoints[keypoint_index];
                    if (mp) {
                        points_compute_center_num++;
                        cv::Mat pos = mp->GetWorldPos();
                        cv::add(centroid, pos, centroid);
                        compute_center = false;
                    }
                }
            }
        }

        float pos_x = centroid.at<float>(0) / (float)points_compute_center_num;
        float pos_y = centroid.at<float>(1) / (float)points_compute_center_num;
        float pos_z = centroid.at<float>(2) / (float)points_compute_center_num;
        centroid = cv::Mat_<float>(3, 1) << pos_x, pos_y, pos_z;

        cv::Mat distance_pos(3, 1, CV_32F);
        for (int j = 0; j < points_in_box[i].size(); ++j) {
            int keypoint_index = points_in_box[i][j];
            MapPoint* mp = mvpMapPoints[keypoint_index];
            if (mp) {
                cv::absdiff(mp->GetWorldPos(), centroid, distance_pos);
                double distance_each_mappoint = norm(distance_pos);
                mappoints_distance_to_center_map.emplace(keypoint_index, distance_each_mappoint);
            }
        }
    }

    bool Frame::ComputeCentroidByDepth(const int& i, cv::Mat& centroid,
                                       map<size_t, double>& mappoints_distance_to_center_map) {
        struct  MapPointDepthCompare {
            bool operator() (const std::pair<float, int>& v1, const std::pair<float, int>& v2) const {
                return v1.first < v2.first;
            }
        };
        vector<double> bbox = objects_cur_[i]->bounding_box_;
        double box_center_x = 0.5 * (bbox[1] + bbox[0]);
        double box_center_y = 0.5 * (bbox[3] + bbox[2]);
        cv::Point2f center_point(box_center_x, box_center_y);
        multiset<std::pair<float, int>, MapPointDepthCompare> mappoints;
        for (int j = 0; j < points_in_box[i].size(); ++j) {
            int keypoint_index = points_in_box[i][j];
            MapPoint* mp = mvpMapPoints[keypoint_index];
            if (mp) {
                cv::Mat pos = mp->GetWorldPos();
                mappoints.emplace(pos.at<float>(2), keypoint_index);
            }
        }
        if (mappoints.size() < 2)
            return false;

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

        //! 计算深度处于中间的点距离box中心最近的点
        float min_distance = 9999.;
        int center_point_index = -1;
        for (int k = 0; k < center_index.size(); ++k) {
            int keypoint_index = center_index[k];
            cv::Point2f kp = mvKeysUn[keypoint_index].pt;
            float distance = norm(kp - center_point);
            if (distance < min_distance) {
                min_distance = distance;
                center_point_index = keypoint_index;
            }
        }
        centroid = mvpMapPoints[center_point_index]->GetWorldPos();

        //! 计算box内的每个点距离中心点的深度距离
        for (int j = 0; j < points_in_box[i].size(); ++j) {
            int keypoint_index = points_in_box[i][j];
            MapPoint* mp = mvpMapPoints[keypoint_index];
            if (mp) {
                cv::Mat pos = mp->GetWorldPos();
                float distance_each_mappoint = fabs(pos.at<float>(2) - centroid.at<float>(2));
                mappoints_distance_to_center_map.emplace(keypoint_index, distance_each_mappoint);
            }
        }
        return true;
    }

    void Frame::GetFrameObject(Map* mpMap) {
        for (int k = 0; k < objects_cur_.size(); ++k) {
            cv::Mat object_pos(3, 1, CV_32F);
            map<size_t, double> mappoints_distance_to_center_map;
            if (!ComputeCentroidByDepth(k, object_pos, mappoints_distance_to_center_map))
                continue;

            objects_cur_[k]->Pos_ = object_pos;
            for (auto iter = mappoints_distance_to_center_map.begin(); iter != mappoints_distance_to_center_map.end(); ++iter) {
                if ((*iter).second > 2.)
                    continue;
                int map_point_id = (*iter).first;
                objects_cur_[k]->MapPoints_index.insert(map_point_id);
                mappoint_mapping_to_object_[map_point_id] = k;
                objects_cur_[k]->MapPonits.push_back(mvpMapPoints[map_point_id]);
            }
//            if (objects_cur_[k]->dynamic_)
//                continue;

            if (objects_cur_[k]->MapPonits.size() < 5)
                continue;
            int same_class_num = 0;
            int repeat_object_num = 0;
            bool continue_to_find = true;
            for (int i = mpMap->objects_in_map_.size() - 1; (i >= 0) && (continue_to_find == true); i--) {
                if (mpMap->objects_in_map_[i] == NULL)
                    continue;
                if (mpMap->objects_in_map_[i]->Pos_.empty())
                    continue;
                if (objects_cur_[k]->class_id_ == mpMap->objects_in_map_[i]->class_id_) {
                    same_class_num++;
                    float x = object_pos.at<float>(0, 0) -
                              mpMap->objects_in_map_[i]->Pos_.at<float>(0, 0);
                    float y = object_pos.at<float>(1, 0) -
                              mpMap->objects_in_map_[i]->Pos_.at<float>(1, 0);
                    float z = object_pos.at<float>(2, 0) -
                              mpMap->objects_in_map_[i]->Pos_.at<float>(2, 0);
                    if (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) > 10) {
                        continue;
                    } else {
                        repeat_object_num++;
                        /* 融合地图点
                        bool new_mappoint = true;
                        for (int j = 0; j < objects_cur_[k]->MapPonits.size(); ++j) {
                            cv::Mat pos_cur = objects_cur_[k]->MapPonits[j]->GetWorldPos();
                            for (int l = 0; l < mpMap->objects_in_map_[i]->MapPonits.size(); ++l) {
                                cv::Mat pos_map = mpMap->objects_in_map_[i]->MapPonits[l]->GetWorldPos();
                                if (cv::countNonZero(pos_cur - pos_map) == 0) {
                                    new_mappoint = false;
                                    break;
                                }
                            }
                            if (new_mappoint) {
                                mpMap->objects_in_map_[i]->MapPonits.push_back(objects_cur_[k]->MapPonits[j]);
                            }
                        }
                         */
                        objects_cur_[k]->mnId_ = mpMap->objects_in_map_[i]->mnId_;
                        if (mpMap->objects_in_map_[i]->last_add_Id != mnId) {
                            mpMap->objects_in_map_[i]->last_add_Id = mnId;
                            mpMap->objects_in_map_[i]->observe_times_++;
                        }
                        continue_to_find = false;
                    }
                }
                else
                    continue;
            }
            if ((same_class_num == 0 || repeat_object_num == 0)) {//! 出现新物体
                objects_cur_[k]->mnId_ = mpMap->objects_in_map_.size();
                objects_cur_[k]->add_frame_id = mnId;
                objects_cur_[k]->last_add_Id = mnId;
                objects_cur_[k]->observe_times_++;
                mpMap->objects_in_map_.push_back(objects_cur_[k]);
            }
        }
    }

    void Frame::ComputeBoxCenter(vector<cv::Point2f>& box_center_vec) {
        for (int i = 0; i < objects_cur_.size(); ++i) {
            vector<double> box = objects_cur_[i]->bounding_box_;
            double left = box[0];
            double right = box[1];
            double top = box[2];
            double bottom = box[3];
            cv::Point2f center((left + right) / 2., (top + bottom) / 2.);
            box_center_vec.push_back(center);
        }
    }

    bool Frame::IsInBox(const int& i, int& box_id) {
        const cv::KeyPoint& kp = mvKeysUn[i];
        float kp_u  = kp.pt.x;
        float kp_v = kp.pt.y;
        bool in_box = false;
        for (int k = 0; k < objects_cur_.size(); ++k) {
            vector<double> box = objects_cur_[k]->bounding_box_;
            double left = box[0];
            double right = box[1];
            double top = box[2];
            double bottom = box[3];
            if (kp_u > left - 2 && kp_u < right + 2
                && kp_v > top - 2 && kp_v < bottom + 2) {
                in_box = true;
                box_id = k;
                break;
            }
        }
        return in_box;
    }

    bool Frame::IsInTrackBox(const int& i, int& box_id) {
        const cv::KeyPoint& kp = mvKeysUn[i];
        float kp_u  = kp.pt.x;
        float kp_v = kp.pt.y;
        bool in_box = false;
        for (int k = 0; k < tracking_object_box_.size(); ++k) {
            vector<double> box = tracking_object_box_[k];
            double left = box[0];
            double right = box[1];
            double top = box[2];
            double bottom = box[3];
            if (kp_u > left - 2 && kp_u < right + 2
                && kp_v > top - 2 && kp_v < bottom + 2) {
                in_box = true;
                box_id = k;
                break;
            }
        }
        return in_box;
    }

    bool Frame::DrawBox(const vector<bool>& is_dynamic, cv::Mat& image) {
        for (int j = 0; j < objects_cur_.size(); ++j) {
            vector<double> box = objects_cur_[j]->bounding_box_;
            double left = box[0];
            double right = box[1];
            double top = box[2];
            double bottom = box[3];
            cv::Point2f p1(left, top);
            cv::Point2f p2(right, bottom);
            cv::Point2f p1_text(left, top - 20);
            cv::Point2f p2_text(left + 30, top);
            cv::rectangle(image, p1_text, p2_text, cv::Scalar(0, 255, 0), -1);
            string mnid_str = to_string(objects_cur_[j]->mnId_);
            cv::putText(image, mnid_str, cv::Point2f(left + 8, top - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 0));
            if (is_dynamic[j]) {
                objects_cur_[j]->dynamic_ = true;
//                cv::rectangle(image, p1, p2, cv::Scalar(0, 0, 255));
            } else {
//                cv::rectangle(image, p1, p2, cv::Scalar(0, 255, 0));
            }
        }
        for (int i = 0; i < tracking_object_box_.size(); ++i) {
            vector<double> box = tracking_object_box_[i];
            cv::Point2f p1(box[0], box[2]);
            cv::Point2f p2(box[1], box[3]);
            cv::rectangle(image, p1, p2, cv::Scalar(0, 255, 0));
        }
    }

} //namespace ORB_SLAM
