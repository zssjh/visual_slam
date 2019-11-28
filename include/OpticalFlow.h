//
// Created by zss on 2019/11/20.
//

#ifndef ORB_SLAM2_OPTICALFLOW_H
#define ORB_SLAM2_OPTICALFLOW_H

#include <iostream>
#include <opencv2/core/types.hpp>
#include "Frame.h"

using namespace std;
namespace ORB_SLAM2 {
class OpticalFlow{
public:
    OpticalFlow(){ init_ = true;}
    void Init(const Frame& LastFrame, const map<int, int>& points_and_box_id);
    double ComputeDistance(const cv::Point2f &pt1, const cv::Point2f &pt2);
    void SetPrediction(const int& id);
    void TrackImage();
    void AddPoint(const cv::Point2f& p1, cv::Point2f& p2);
    void AvgPoint(const int& n, cv::Point2f& p);

public:
    bool init_;
    unsigned int points_size;
    unsigned int object_size;
    Frame LastFrame_, CurrentFrame_;
    vector<vector<cv::Point2f>> prev_pts_, cur_pts_, predict_pts_;
    vector<cv::Point2f> box_center_motion;
    vector<bool> valid_tracker_;
    bool hasPrediction_, flowBack_;
    vector<vector<std::pair<cv::Point2f, cv::Point2f>>> optical_flow_points_pairs_;
};

}
#endif //ORB_SLAM2_OPTICALFLOW_H
