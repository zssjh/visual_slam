//
// Created by zss on 19-6-6.
//

#ifndef ORB_SLAM2_FUNDAMENTAL_H
#define ORB_SLAM2_FUNDAMENTAL_H

#include <vector>
#include <opencv2/core/mat.hpp>
using namespace std;

void Cal_Fundamental(const vector<cv::KeyPoint>& last,const vector<cv::KeyPoint>& cur,vector<bool> &vbMatchesInliers, cv::Mat &F21);
cv::Mat Compute_F21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2);
float Check_Fundamental(const vector<cv::KeyPoint>& last, const vector<cv::KeyPoint>& cur, const cv::Mat &F21, vector<bool> &vbMatchesInliers);
void Normalize(const vector<cv::KeyPoint>& last,const vector<cv::KeyPoint>& cur,const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
#endif //ORB_SLAM2_FUNDAMENTAL_H

