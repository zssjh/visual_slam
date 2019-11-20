//
// Created by zss on 18-4-19.
//
#include "Object.h"
#include "vector"
#include "math.h"
#define PI 3.14159265

namespace ORB_SLAM2 {
    Object::Object() {
        dynamic_ = false;
        mnId_ = -1;
    }
    Object::~Object()
    {}
    void Object::UpdateCubeSize() {
//        unique_lock<mutex> lock(mMutexUpdateCube);
        float sum_x = 0;
        float sum_y = 0;
        float sum_z = 0;
        vector<float> x_pt;
        vector<float> y_pt;
        vector<float> z_pt;

        if (MapPonits.size() < 10) {
            mCubeBox = (cv::Mat_<double>::zeros(10, 1));
            return;
        }

        for (int j = 0; j < MapPonits.size() ; ++j) {
            MapPoint *pp = MapPonits[j];
            cv::Mat pp_pt = pp->GetWorldPos();
        
            sum_x += pp_pt.at<float>(0);
            sum_y += pp_pt.at<float>(1);
            sum_z += pp_pt.at<float>(2);
        
            x_pt.push_back(pp_pt.at<float>(0));
            y_pt.push_back(pp_pt.at<float>(1));
            z_pt.push_back(pp_pt.at<float>(2));
        }
        // calculate cube center
        float x_co = sum_x / MapPonits.size();
        float y_co = sum_y / MapPonits.size();
        float z_co = sum_z / MapPonits.size();
    
        // calculate cube boundary
        sort(x_pt.begin(), x_pt.end());
        sort(y_pt.begin(), y_pt.end());
        sort(z_pt.begin(), z_pt.end());
        float x_min=x_pt[0];
        float x_max=x_pt[x_pt.size()-1];
        float y_min=y_pt[0];
        float y_max=y_pt[y_pt.size()-1];
        float z_min=z_pt[0];
        float z_max=z_pt[z_pt.size()-1];
        float volume = (x_max-x_min) * (y_max-y_min) * (z_max-z_min);
        mCubeBox = (cv::Mat_<double>(10, 1) <<
                                           x_co, y_co, z_co,
                                           x_min, x_max,
                                           y_min, y_max,
                                           z_min, z_max,
                                           volume);
    }
    
    cv::Mat Object::GetCubeBox() {
//        unique_lock<mutex> lock(mMutexUpdateCube);
        return mCubeBox;
    }
    
    double Object::ComputeIoU(shared_ptr<Object> pTargetObj) {
        UpdateCubeSize();
        pTargetObj->UpdateCubeSize();
        if (cv::countNonZero(mCubeBox) == 0 || cv::countNonZero(pTargetObj->mCubeBox) == 0) {
            return 0.;
        }
        // no cross return
        cv::Mat targetCubeBox = pTargetObj->GetCubeBox();

        // cross part vertex
        double x_min_cross = max (mCubeBox.at<double>(3, 0), targetCubeBox.at<double>(3, 0));
        double x_max_cross = min (mCubeBox.at<double>(4, 0), targetCubeBox.at<double>(4, 0));
        double y_min_cross = max (mCubeBox.at<double>(5, 0), targetCubeBox.at<double>(5, 0));
        double y_max_cross = min (mCubeBox.at<double>(6, 0), targetCubeBox.at<double>(6, 0));
        double z_min_cross = max (mCubeBox.at<double>(7, 0), targetCubeBox.at<double>(7, 0));
        double z_max_cross = min (mCubeBox.at<double>(8, 0), targetCubeBox.at<double>(8, 0));

        // cross part edge
        double x_edge_cross = x_max_cross - x_min_cross;
        double y_edge_cross = y_max_cross - y_min_cross;
        double z_edge_cross = z_max_cross - z_min_cross;

        // double check no cross
        if (x_edge_cross < 0 || y_edge_cross < 0 || z_edge_cross < 0) {
            return 0.;
        }

        // cross_volume / curr_volume
        double iou = (x_edge_cross*y_edge_cross*z_edge_cross) / mCubeBox.at<double>(9, 0);
        if (std::isnan(iou)) {
            cout << mCubeBox.at<double>(9, 0) << endl;
        }
        return iou;
    }
    
    void Object::FuseObj(shared_ptr<Object> pTargetObj) {
        last_add_Id = pTargetObj->last_add_Id;
        MapPonits.insert(MapPonits.end(), pTargetObj->MapPonits.begin(), pTargetObj->MapPonits.end());
        co_MapPonits.insert(co_MapPonits.end(), pTargetObj->co_MapPonits.begin(), pTargetObj->co_MapPonits.end());
        UpdateCubeSize();
    }
}

