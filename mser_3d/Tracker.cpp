//
// Created by alex on 4/22/16.
//

#include "Tracker.h"
#include <opencv2/core/core.hpp> //need for reading YAML files
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h> //need for mkdir
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace gtsam;

Tracker::Tracker(InputManager& input){
    input_ = input;
    readImages();
    observeMSERs();
}

void Tracker::readImages() {
    //Extract video frames and store in memory
    cv::VideoCapture capture(input_.videoPath());
    //std::vector<cv::Mat> allVideoFrames;
    int f = 0;
    if (!capture.isOpened()) {
        cerr << "The video file could not be opened successfully!!!" << endl;
    } else {
        bool readSuccess = true;
        while (readSuccess == true) {
            cv::Mat videoFrame;
            readSuccess = capture.read(videoFrame);
            if (f >
                13) { //remove first 14 frames because Matlab and OpenCV don't open the same video in the same way :(
                images_.push_back(videoFrame);
            }
            f++;
        }
    }
    capture.release();
}

void Tracker::observeMSERs() {
    //Observe MSERs and place results into Frame data structure.
    for (int f = 0; f < images_.size(); f++) {
        cv::MSER mser(input_.delta(),
                      input_.minArea(),
                      input_.maxArea(),
                      input_.maxVariation(),
                      input_.minDiversity(),
                      input_.maxEvolution(),
                      input_.areaThreshold(),
                      input_.minMargin(),
                      input_.edgeBlurSize()); //initialize MSER
        vector<vector<cv::Point>> regions; //data structure to store pixels for each region
        cv::Mat image = images_[f]; //grab image
        cv::Mat gray;
        cv::cvtColor(image, gray, CV_BGR2GRAY ); //convert to gray
        const cv::Mat mask;
        mser(gray, regions, mask); //Detect MSER on grayscale image from file. Store results in regions vector.
        std::vector<MserMeasurementColor> measurements;
        for (size_t i = 0; i < regions.size(); i++)
        {
            cv::RotatedRect rr = fitEllipse(regions[i]); //fit ellipse to region. Store info in rotated rectangle data structure
            ellipse(gray, rr, cv::Scalar(0,0,0),5);
            Pose2 ellipsePose = Pose2(rr.center.x,rr.center.y,rr.angle);
            Point2 ellipseAxes = Point2(rr.size.height,rr.size.width);
            MserMeasurement msmt = MserMeasurement(ellipsePose,ellipseAxes);
            cv::Vec3b color = image.at<cv::Vec3b>(rr.center.y, rr.center.x);
            MserMeasurementColor msmtColor = MserMeasurementColor(msmt,color);
            measurements.push_back(msmtColor);
        }
        //Frame frame = Frame(f,);
    }
}

void Tracker::writeImages() const{
    //Save tracker images to disk
    int imgDirectorySuccess = mkdir(input_.imagePath().c_str(), 0777);
    for (int f = 0; f < images_.size(); f++){
        char imgFileName[200];
        cv::Mat videoFrame = images_[f];
        strcpy(imgFileName,input_.imagePath().c_str());
        strcat(imgFileName,"Frame%04i.bmp");
        sprintf(imgFileName, imgFileName,f);
        cv::imwrite(imgFileName, videoFrame);
    }
}


