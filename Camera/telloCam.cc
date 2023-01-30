#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp>
#include<opencv2/imgcodecs.hpp>

#include<System.h>
#include<Converter.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc != 3) {
        cerr << endl << "argc:" << argc << "!= 3"<< endl;
    }
    
    cv::VideoCapture cap{"udp://192.168.10.1:11111?overrun_nonfatal=1&fifo_size=5000", cv::CAP_FFMPEG};
	
    if (!cap.isOpened()) {
        cerr << endl << "Could not open camera feed; Reattempting... (telloCam.cc)" << endl;
        sleep(2);
        cap.open("udp://0.0.0.0:11111?overrun_nonfatal=1&fifo_size=5000", cv::CAP_FFMPEG);
        
        if(!cap.isOpened()){
            cerr << "Could not open camera feed; Ending program... (telloCam.cc)" << endl;
            return -1;
        }
    }
        
    //Loop until camera feed is found
    cv::Mat frame;
    while(true){
        sleep(1);
        cap >> frame;
        if(!frame.empty()){
            break;
        }
        cout << "Finding Feed" << endl;
    }
    
    cout << "Feed found" << endl;

    return 0;
}
