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

#ifndef FRAME_H
#define FRAME_H

#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "MapPoint.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Object.h"
#include "Map.h"

namespace ORB_SLAM2 {
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class Object;
class Map;
class Frame {
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp,
          ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc,
          cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth,
          const vector<std::pair<vector<double>, int>>& bounding_box);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,
            ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc,
            cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    cv::Mat UnprojectStereo(const int &i);
    void ComputeCentroid(const vector<unsigned int>& all_mappoints, cv::Mat& centroid, map<size_t, double>& mappoints_distance_to_center_map);
    void ComputeCentroid2D(const int& i, cv::Mat& centroid, map<size_t, double>& mappoints_distance_to_center_map);
    bool ComputeCentroidByDepth(const int& i, cv::Mat& centroid,
                                       map<size_t, double>& mappoints_distance_to_center_map);
    void GetFrameObject(Map* mpMap);
    bool IsInDynamicBox(const int& i);
    void ComputeBoxCenter(vector<cv::Point2f>& box_center_vec);
    bool IsInBox(const int& i, int& box_id);
    bool IsInTrackBox(const int& i, int& box_id, int& class_id);
    bool DrawBox(const vector<bool>& is_dynamic, cv::Mat& image);
    bool DrawBoxPredict(const vector<cv::Point2f>& box_center);
    cv::Mat GetMtw() { return mtcw; }
    cv::Mat GetMRw() { return mRcw; }
    cv::Mat skew(const cv::Mat& t) {
        cv::Mat skew_t = cv::Mat::zeros(3, 3, CV_32F);
        if (t.rows != 3 || t.cols != 1)
            return skew_t;
        skew_t.at<float>(0, 1) = -t.at<float>(2);
        skew_t.at<float>(0, 2) = t.at<float>(1);
        skew_t.at<float>(1, 2) = -t.at<float>(0);
        skew_t.at<float>(1, 0) = t.at<float>(2);
        skew_t.at<float>(2, 0) = -t.at<float>(1);
        skew_t.at<float>(2, 1) = t.at<float>(0);
        return skew_t;
    }

public:
    bool read_detect_state_;
    cv::Mat optical_flow_image_;
    vector<cv::Point2f> predict_box_center_vec;
    map<int, std::pair<int, int>> points_for_optical_flow;
    map<int, int> points_optical_flow_succes;
    map<int, int> matches_out_box;
    map<int, std::pair<int, int>> matches_in_box;
    vector<vector<int>> points_in_box;
    //! vector num = objects
    vector<vector<double>> tracking_object_box_;
    vector<int> tracking_object_box_class_;
    vector<bool> valid_tracker_;
    vector<cv::Point2f> tracking_object_motion_;
    vector<vector<std::pair<cv::Point2f, cv::Point2f>>> optical_flow_points_pairs_;
    vector<std::pair<cv::Point2f, cv::Point2f>> optical_flow_points_pairs_filter_;
    //!
    vector<std::shared_ptr<Object>> objects_cur_detect_;
    vector<std::shared_ptr<Object>> objects_cur_;
    cv::Mat current_frame_image, current_frame_image_log;
    map<unsigned int, unsigned int> mappoint_mapping_to_object_;

    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
