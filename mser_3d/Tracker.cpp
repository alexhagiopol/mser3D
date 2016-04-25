//
// Created by alex on 4/22/16.
//

#include "Tracker.h"
#include <opencv2/core/core.hpp> //need for reading YAML files
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h> //need for mkdir

void Tracker::observeMSERs() {
    //Extract video frames and store in memory
    cv::VideoCapture capture(input_.videoPath());
    std::vector<cv::Mat> allVideoFrames;
    int f = 0;
    if (!capture.isOpened()) {
        cerr << "The video file could not be opened successfully!!!" << endl;
    } else {
        bool readSuccess = true;
        while (readSuccess == true) {
            cv::Mat videoFrame;
            readSuccess = capture.read(videoFrame);
            if (f > 13) { //remove first 14 frames because Matlab and OpenCV don't open the same video in the same way :(
                allVideoFrames.push_back(videoFrame);
            }
            f++;
        }
    }

    capture.release();
    int imgDirectorySuccess = mkdir(input_.imagePath().c_str(), 0777);
    for (int f = 0; f < allVideoFrames.size(); f++){
        char imgFileName[200];
        cv::Mat videoFrame = allVideoFrames[f];
        strcpy(imgFileName,input_.imagePath().c_str());
        strcat(imgFileName,"Frame%04i.bmp");
        sprintf(imgFileName, imgFileName,f);
        cv::imwrite(imgFileName, videoFrame);
    }
}
