//
// Created by zss on 18-4-19.
//

#ifndef MYSLAM_OBJECT_H
#define MYSLAM_OBJECT_H

#include <fstream>
#include <mutex>
#include <set>
#include "MapPoint.h"
using namespace std;

namespace ORB_SLAM2 {
    class MapPoint;
    class Object {
    public:
        Object();
        ~Object();
    public:
        enum classname{
            bus = 0,
            car = 1,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3
        };
        int mnId_;
        unsigned int last_add_Id;
        unsigned int class_id_;
        unsigned int observe_times_;
        unsigned int add_frame_id;
        cv::Mat Pos_;
        cv::Point2f center_point_;
        bool bad_;
        bool dynamic_;
        bool First_obj_;
        bool current_;
        vector<int> bounding_box_;
        struct KeyPointComp {
            bool operator() (const std::pair<cv::KeyPoint, unsigned int>& p1,
                    const std::pair<cv::KeyPoint, unsigned int>& p2) const{
                return p1.first.pt.x > p2.first.pt.x;
            }
        };
        set<unsigned int> MapPoints_index;

        vector< MapPoint*>  MapPonits;  // (Map)地图点, 所有物体范围内
        vector< MapPoint*>  pro_MapPonits;  // 世界坐标系的(Frame)地图点
        vector<cv::Mat> pro_MapPoints_camera;  //! (Frame)地图点的相机位姿, 用来计算和下面世界坐标系地图点的距离
        vector< MapPoint*>  co_MapPonits;  //! (Map)地图点, 在上一帧中也被看到过, 物体质心地图点集合
        
    public:
        void UpdateCubeSize();
        cv::Mat GetCubeBox();
        double ComputeIoU(shared_ptr<Object> pTargetObj);
        void FuseObj(shared_ptr<Object> pTargetObj);

    protected:
        // [
        // x_co, y_co, z_co, [0, 1, 2]
        // x_min, x_max,     [3, 4]
        // y_min, y_max,     [5, 6]
        // z_min, z_max,     [7, 8]
        // volume            [9]
        // ]
        cv::Mat mCubeBox;  // shape = [10, 1]
        mutex mMutexUpdateCube;
    };
    
}
#endif //MYSLAM_OBJECT_H
