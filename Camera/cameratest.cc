/*
* Modified version of https://github.com/waseemtannous/Autonomous-Drone-Scanning-and-Mapping/blob/main/mono_tum.cc
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>

#include<opencv2/core/core.hpp>

#include<System.h>
#include <Converter.h>


using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);


// how to run:
// 0 for webcam, 1 for drone
// ./Examples/Monocular/mono_tum 0 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml
// ./Examples/Monocular/mono_tum 1 Vocabulary/ORBvoc.txt Examples/Monocular/DRONE_PARAMS.yaml

int main(int argc, char **argv)
{

    std::string str0 ("0");
    std::string str1 ("1");

    //Run ORB SLAM with connected computer camera source
    if (str0.compare(argv[1]) == 0) {
        if(argc != 4) {
            cerr << endl << "argc:" << argc << "!= 4"<< endl;
        }

        cv::VideoCapture cap(0);

        if (!cap.isOpened()) {
            cerr << endl << "Could not open camera feed." << endl;
            return -1;
        }
        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::MONOCULAR, true);
        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;


        //Not usable with C++11; use monotonic_clock
        std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();


        // Main loop
        while(true)//cv::waitKey(0) != 27)
        {
            //Create a new Mat
            cv::Mat frame;

            //Send the captured frame to the new Mat
            cap >> frame;

            if(frame.empty())
                break;


            //Not usable with C++11; use monotonic_clock
            std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now();


            // Pass the image to the SLAM system
            SLAM.TrackMonocular(frame, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count());
        }

        // Save points
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetTrackedMapPoints();
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for(auto p : mapPoints) {
            if (p != NULL)
            {
                auto point = p->GetWorldPos();
                //Eigen::Matrix<double, 3, 1> v = ORB_SLAM3::Converter::toVector3d(point);
                Eigen::Matrix<double, 3, 1> v = point.cast<double>();
                pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            }
        }
        pointData.close();
    
        // Stop all threads
        SLAM.Shutdown();

        //slam->SaveSeperateKeyFrameTrajectoryTUM("KeyFrameTrajectory-1.txt", "KeyFrameTrajectory-2.txt", "KeyFrameTrajectory-3.txt");
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    //Run ORB SLAM with drone camera source
    if (str1.compare(argv[1]) == 0) {
        if(argc != 4) {
            cerr << endl << "argc:" << argc << "!= 4"<< endl;
        }

        cv::VideoCapture cap("udp://@0.0.0.0:11111?overrun_nonfatal=1&fifo_size=50000000");

        if (!cap.isOpened()) {
            cerr << endl << "Could not open camera feed." << endl;
            return -1;
        }
        ORB_SLAM3::System SLAM(argv[2], argv[3], ORB_SLAM3::System::MONOCULAR, true);
        cout << endl << "-------" << endl;
        cout << "Start processing sequence ..." << endl;


        //Not usable with C++11; use monotonic_clock
        std::chrono::steady_clock::time_point initT = std::chrono::steady_clock::now();


        // Main loop
        while(true)//cv::waitKey(0) != 27)
        {
            //Create a new Mat
            cv::Mat frame;

            //Send the captured frame to the new Mat
            cap >> frame;

            if(frame.empty())
                break;


            //Not usable with C++11; use monotonic_clock
            std::chrono::steady_clock::time_point nowT = std::chrono::steady_clock::now(); 


            // Pass the image to the SLAM system
            SLAM.TrackMonocular(frame, std::chrono::duration_cast<std::chrono::duration<double> >(nowT-initT).count());
        }

        // Save points
        std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM.GetTrackedMapPoints();
        std::ofstream pointData;
        pointData.open("/tmp/pointData.csv");
        for(auto p : mapPoints) {
            if (p != NULL)
            {
                auto point = p->GetWorldPos();
                //Eigen::Matrix<double, 3, 1> v = ORB_SLAM3::Converter::toVector3d(point);
                Eigen::Matrix<double, 3, 1> v = point.cast<double>();
                pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
            }
        }
        pointData.close();
    
        // Stop all threads
        SLAM.Shutdown();


        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}