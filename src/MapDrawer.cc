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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{
MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();
    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++) {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for (set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++) {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();
}

void MapDrawer::DrawObjectMapPoints() {
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for (int j = 0; j < mpMap->objects_in_map_.size(); ++j) {
        if (mpMap->objects_in_map_[j] == NULL)
            continue;
        for (int i = 0; i < mpMap->objects_in_map_[j]->MapPonits.size(); ++i) {
            cv::Mat pos = mpMap->objects_in_map_[j]->MapPonits[i]->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
        }
    }
    glEnd();
}

/**
 * @brief 连接当前可见物体之间的连线
 * @param Twc
 */
void MapDrawer::DrawLineBetweenObjects(pangolin::OpenGlMatrix &Twc) {
    map<unsigned int, cv::Mat> draw_line_between_objects_cur;
    for (size_t i = 0; i < mpMap->objects_in_map_.size(); i++) {
        if (mpMap->objects_in_map_[i] == NULL)
            continue;
         if (mpMap->objects_in_map_[i]->dynamic_) {
             continue;
         }

        mpMap->objects_in_map_[i]->UpdateCubeSize();
        cv::Mat cubeBox = mpMap->objects_in_map_[i]->GetCubeBox();
        float x_center = mpMap->objects_in_map_[i]->Pos_.at<float>(0);
        float y_center = mpMap->objects_in_map_[i]->Pos_.at<float>(1);
        float z_center = mpMap->objects_in_map_[i]->Pos_.at<float>(2);

        float x_min = x_center - 2;
        float x_max = x_center + 2;
        float y_min = y_center - 1;
        float y_max = y_center + 1;
        float z_min = z_center - 2;
        float z_max = z_center + 2;

        glPointSize(4 * mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex3f(x_center, y_center, z_center);
        glEnd();

        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f);
        glVertex3f(x_min,y_min,z_min);
        glVertex3f(x_max,y_min,z_min);
        glVertex3f(x_min,y_min,z_min);
        glVertex3f(x_min,y_max,z_min);
        glVertex3f(x_min,y_min,z_min);
        glVertex3f(x_min,y_min,z_max);

        glVertex3f(x_max,y_max,z_min);
        glVertex3f(x_max,y_min,z_min);
        glVertex3f(x_max,y_max,z_min);
        glVertex3f(x_min,y_max,z_min);
        glVertex3f(x_max,y_max,z_min);
        glVertex3f(x_max,y_max,z_max);

        glVertex3f(x_max,y_min,z_max);
        glVertex3f(x_max,y_min,z_min);
        glVertex3f(x_max,y_min,z_max);
        glVertex3f(x_min,y_min,z_max);
        glVertex3f(x_max,y_min,z_max);
        glVertex3f(x_max,y_max,z_max);

        glVertex3f(x_min,y_max,z_max);
        glVertex3f(x_min,y_min,z_max);
        glVertex3f(x_min,y_max,z_max);
        glVertex3f(x_min,y_max,z_min);
        glVertex3f(x_min,y_max,z_max);
        glVertex3f(x_max,y_max,z_max);
        glEnd();

        glColor3f(0, 0, 1);
        string mnId_str = to_string(mpMap->objects_in_map_[i]->mnId_);
        pangolin::GlFont::I().Text(mnId_str).Draw(x_center + 1, y_center + 1, z_center+ 1);

        if (Frame::nNextId - 1 != mpMap->objects_in_map_[i]->last_add_Id)
            continue;
        draw_line_between_objects_cur.emplace(i, mpMap->objects_in_map_[i]->Pos_);
    }
    draw_line_between_objects_.push_back(draw_line_between_objects_cur);
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(1.0f,1.0f,0.0f);
    glBegin(GL_LINES);

    for (int j = 0; j < draw_line_between_objects_.size(); ++j) {
        for (auto iter1 = draw_line_between_objects_[j].begin(); iter1 != draw_line_between_objects_[j].end() ; ++iter1) {
            if (mpMap->objects_in_map_[iter1->first] == NULL)
                continue;
            for (auto iter2 = draw_line_between_objects_[j].begin(); iter2 != draw_line_between_objects_[j].end() ; ++iter2) {
                if (mpMap->objects_in_map_[iter2->first] == NULL)
                    continue;
                if (iter1->first == iter2->first)
                    continue;
                glVertex3f(iter1->second.at<float>(0), iter1->second.at<float>(1), iter1->second.at<float>(2));
                glVertex3f(iter2->second.at<float>(0), iter2->second.at<float>(1), iter2->second.at<float>(2));
            }
        }
    }
    glEnd();
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    if (bDrawKF) {
        for (size_t i=0; i<vpKFs.size(); i++) {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));
            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
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
    }

    if (bDrawGraph) {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for (size_t i=0; i<vpKFs.size(); i++) {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if (!vCovKFs.empty()) {
                for (vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++) {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }
            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if (pParent) {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }
            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for (set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++) {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }
        glEnd();
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

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
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
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
