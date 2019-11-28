//
// Created by zss on 2019/11/20.
//
#include "OpticalFlow.h"

namespace ORB_SLAM2{
void OpticalFlow::Init(const Frame& LastFrame, const map<int, std::pair<int, int>>& points_and_box_id) {
    LastFrame_ = LastFrame;
    object_size = LastFrame.tracking_object_box_.size();
    prev_pts_.clear();
    prev_pts_.resize(object_size);
    predict_pts_.clear();
    predict_pts_.resize(object_size);
    cur_pts_.clear();
    cur_pts_.resize(object_size);
    box_center_motion.clear();
    box_center_motion.resize(object_size, cv::Point2f(0, 0));
    box_class_.clear();
    box_class_.resize(object_size, -1);
    valid_tracker_.clear();
    valid_tracker_.resize(object_size, true);
    optical_flow_points_pairs_.clear();
    optical_flow_points_pairs_.resize(object_size);
    for (auto iter = points_and_box_id.begin(); iter != points_and_box_id.end(); iter++) {
        int point_id = iter->first;
        int box_id = iter->second.first;
        box_class_[box_id] = iter->second.second;
        prev_pts_[box_id].push_back(LastFrame.mvKeysUn[point_id].pt);
    }
    hasPrediction_ = false;
    flowBack_ = true;
}

double OpticalFlow::ComputeDistance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

void OpticalFlow::SetPrediction(const int& id) {
    predict_pts_[id].clear();
    predict_pts_[id].resize(points_size);
    for (int i = 0; i < points_size; ++i) {
        if (LastFrame_.mvDepth[i] > 0) {
            //! from last frame to world
            cv::Mat point_last_pixel, point_last_normalize, point_last_camera, point_last_camera_homo;
            cv::Mat point_world, point_cur_pixel, point_cur_normalize, point_cur_camera, point_cur_camera_homo;
            point_last_pixel = cv::Mat_<float>(3, 1) << prev_pts_[id][i].x, prev_pts_[id][i].y, 1;
            point_last_normalize = LastFrame_.mK * point_last_pixel;
            point_last_camera = LastFrame_.mvDepth[i] * point_last_normalize;
            point_last_camera_homo = cv::Mat_<float>(4, 1) <<
                    point_last_camera.at<float>(0),
                    point_last_camera.at<float>(1),
                    point_last_camera.at<float>(2), 1;
            point_world = LastFrame_.mTcw.inv() * point_last_camera_homo;

            //! from world to current frame
            point_cur_camera_homo = CurrentFrame_.mTcw * point_world;
            float scale_w = point_cur_camera_homo.at<float>(3);//! 1
            point_cur_camera = cv::Mat_<float>(3, 1) <<
                    point_cur_camera_homo.at<float>(0)/ scale_w,
                    point_cur_camera_homo.at<float>(1)/ scale_w,
                    point_cur_camera_homo.at<float>(2)/ scale_w;
            float scale_c = point_cur_camera.at<float>(3);
            point_cur_normalize = cv::Mat_<float>(3, 1) <<
                    point_cur_camera.at<float>(0)/ scale_c,
                    point_cur_camera.at<float>(1)/ scale_c,
                    point_cur_camera.at<float>(2)/ scale_c;
            point_cur_pixel = CurrentFrame_.mK * point_cur_camera;

            //! set points and status
            predict_pts_[id][i] = cv::Point2f(point_cur_pixel.at<float>(0), point_cur_pixel.at<float>(1));
        }
    }
}

void OpticalFlow::AddPoint(const cv::Point2f& p1, cv::Point2f& p2) {
    p2.x = p1.x + p2.x;
    p2.y = p1.y + p2.y;
}

void OpticalFlow::AvgPoint(const int& n, cv::Point2f& p) {
    p.x = p.x / n;
    p.y = p.y / n;
}

void OpticalFlow::TrackImage() {
    for (int i = 0; i < object_size; ++i) {
        if (prev_pts_[i].size() > 0) {
            vector<uchar> status;
            vector<float> err;
            if (hasPrediction_) {
                SetPrediction(i);
                cur_pts_ = predict_pts_;
                cv::calcOpticalFlowPyrLK(LastFrame_.current_frame_image, CurrentFrame_.current_frame_image, prev_pts_[i], cur_pts_[i],
                                         status, err, cv::Size(15, 15), 2, cv::TermCriteria(cv::TermCriteria::
                                                                                            COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            } else {
                cv::calcOpticalFlowPyrLK(LastFrame_.current_frame_image, CurrentFrame_.current_frame_image, prev_pts_[i],
                                         cur_pts_[i], status, err, cv::Size(21, 21), 3);
            }

            // reverse check
            if (flowBack_) {
                vector<uchar> reverse_status;
                vector<cv::Point2f> reverse_pts = prev_pts_[i];
                cv::calcOpticalFlowPyrLK(CurrentFrame_.current_frame_image, LastFrame_.current_frame_image, cur_pts_[i],
                                         reverse_pts, reverse_status, err, cv::Size(15, 15), 2, cv::TermCriteria(cv::TermCriteria::
                                         COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
                for (size_t j = 0; j < status.size(); j++) {
                    if (status[j] && reverse_status[j] && ComputeDistance(prev_pts_[i][j], reverse_pts[j]) <= 0.5) {
                        status[j] = 1;
                    } else {
                        status[j] = 0;
                    }
                }
            }

            int points = 0;
            for (int j = 0; j < prev_pts_[i].size(); ++j) {
                if (status[j] == 1) {
                    points++;
                    AddPoint(cur_pts_[i][j] - prev_pts_[i][j], box_center_motion[i]);
                    optical_flow_points_pairs_[i].emplace_back(prev_pts_[i][j], cur_pts_[i][j]);
                }
            }

            //! 框内既没有光流点，也没有上一帧的运动状态，直接不对其进行跟踪
            if (LastFrame_.read_detect_state_) {
                if (points == 0) {
                    valid_tracker_[i] = false;
                    cout << "invalid" << endl;
                } else {
                    AvgPoint(points, box_center_motion[i]);
                }
            } else {
                if (points > 3) {
                    AvgPoint(points, box_center_motion[i]);
                } else {
                    box_center_motion[i] = LastFrame_.tracking_object_motion_[i];
                }
            }
        }
    }
}
}

