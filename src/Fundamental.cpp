//
// Created by zss on 19-6-6.
//

#include <Thirdparty/DBoW2/DUtils/Random.h>
#include <opencv2/core.hpp>
#include <iostream>
#include "Fundamental.h"

using namespace std;
cv::Mat Compute_F21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2) {
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}

float Check_Fundamental(const vector<cv::KeyPoint>& last, const vector<cv::KeyPoint>& cur, const cv::Mat &F21, vector<bool> &vbMatchesInliers) {
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    int N = last.size();
    vbMatchesInliers.resize(N);
    int inlier = 0;

    float th = 5.;
    for (int i = 0; i < N; i++) {
        bool bIn = true;
        const cv::KeyPoint &kp1 = last[i];
        const cv::KeyPoint &kp2 = cur[i];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2 * u2 + b2 * v2 + c2;
        const float Dist1 = fabs(num2) / sqrt(a2 * a2 + b2 * b2);

        if(Dist1 > th)
            bIn = false;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1 * u1 + b1 * v1 + c1;
        const float Dist2 = fabs(num1) / sqrt(a1 * a1 + b1 * b1);

        if(Dist2 > th)
            bIn = false;

        if (bIn) {
            inlier++;
            vbMatchesInliers[i] = true;
        } else {
            vbMatchesInliers[i] = false;
        }
    }
    return inlier;
}

void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

void Cal_Fundamental(const vector<cv::KeyPoint>& last,const vector<cv::KeyPoint>& cur, vector<bool> &vbMatchesInliers, cv::Mat &F21)
{
    // Number of putative matches
    const int N = last.size();
    vector<int> vAllIndices;
    vAllIndices.reserve(N);
    vector<int> vAvailableIndices;
    for (int i = 0; i < N; i++) {
        vAllIndices.push_back(i);
    }

    vector<vector<int> > mvSets;
    mvSets = vector<vector<int> >(200,vector<int>(8,0));
    DUtils::Random::SeedRandOnce(0);

    for (int it = 0; it < 200; it++) {
        vAvailableIndices = vAllIndices;
        for (size_t j = 0; j < 8; j++) {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            int idx = vAvailableIndices[randi];
            mvSets[it][j] = idx;
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(last,vPn1, T1);
    Normalize(cur,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    float score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for (int it = 0; it < 200; it++) {
        // Select a minimum set
        for (int j = 0; j < 8; j++) {
            int idx = mvSets[it][j];
            vPn1i[j] = vPn1[idx];
            vPn2i[j] = vPn2[idx];
        }
        cv::Mat Fn = Compute_F21(vPn1i,vPn2i);
        F21i = T2t*Fn*T1;
        currentScore = Check_Fundamental(last, cur, F21i, vbCurrentInliers);
        if (currentScore > score) {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

