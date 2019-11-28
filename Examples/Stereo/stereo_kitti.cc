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

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result);

int main(int argc, char **argv) {
    if(argc != 5) {
        cerr << endl << "Usage: "
                        "./stereo_kitti "
                        "path_to_vocabulary "
                        "path_to_settings "
                        "path_to_sequence"
                        "path_to_detection_result"
                        << endl;
        return 1;
    }
    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   

    // Main loop
    cv::Mat imLeft, imRight;
    for (int ni = 0; ni < nImages; ni++) {
//        cv::waitKey(0);
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        if (imLeft.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }
        vector<std::pair<vector<double>, int>> detection_box;
        LoadBoundingBox(string(argv[4]) + to_string(ni+1) + ".txt", detection_box);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft, imRight, tframe, detection_box);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for (int ni=0; ni<nImages; ni++) {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("/home/zss/Documents/trajectory/03_static.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void LoadBoundingBox(const string& strPathToDetectionResult, vector<std::pair<vector<double>, int>>& detect_result) {
    ifstream infile;
    infile.open(strPathToDetectionResult);
    if (!infile.is_open()) {
        cout<<"yolo_detection file open fail"<<endl;
        exit(233);
    }
    vector<double> row;
    string line;
    while (getline(infile, line)) {
        int sum = 0, num_bit = 0;
        string obj = "obj=";
        int idx = line.find(obj);
        string class_label;
        for (int j = idx + 4; j < line.size() - 1; ++j) {
            class_label += line[j];
        }
        for (char c : line) {
            if (c >= '0' && c <= '9') {
                num_bit = c - '0';
                sum = sum * 10 + num_bit;
            } else if (c == ',') {
                row.push_back(sum);
                sum = 0;
                num_bit = 0;
            }
        }
        int class_id = -1;
        if (class_label == "bus") {
            class_id = 0;
        } else if (class_label == "car") {
            class_id = 1;
        } else if (class_label == "bicycle") {
            class_id = 2;
        } else if (class_label == "person") {
            class_id = 3;
        } else if (class_label == "truck") {
            class_id = 4;
        }
        detect_result.emplace_back(row, class_id);
        row.clear();
        line.clear();
    }
    infile.close();
}
