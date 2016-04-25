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

void Tracker::readImages() { //TODO: Maybe move to InputManager?
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
    int cameraPoseIndex = 0; //Assumes that first image in images_ matches first pose in input_.VOCameraPoses() TODO: Better alignment of pose data and image data
    std::vector<Pose3> cameraPoses;
    input_.VOCameraPoses(cameraPoses);
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
            //cv::Vec3b color = image.at<cv::Vec3b>(rr.center.y, rr.center.x);
            cv::Scalar color = image.at<cv::Scalar>(rr.center.y, rr.center.x);
            MserMeasurementColor msmtColor = MserMeasurementColor(msmt,color);
            measurements.push_back(msmtColor);
        }
        if (f < cameraPoses.size()){ //If we have more images than camera poses, tough luck. We won't make more Frame data structures. See TODO above.
            Frame frame = Frame(f,cameraPoses[f],measurements);
            frames_.push_back(frame);
            cameraPoseIndex ++;
        }
    }
}

void Tracker::testFrameObservations() {
    //Draw MserMeasurement ellipses on each video frame
    std::vector<cv::Mat> outputImages;
    for (int f = 0; f < frames_.size(); f++){ //loop through all frames
        Frame frame = frames_[f];
        cv::Mat myImage = frame.image_;
        for (int m = 0; m < frames_[f].measurementsColor_.size(); m++){ //loop through all measurements in frame
            //Extract measurement data
            MserMeasurementColor msmtColor = frame.measurementsColor_[m];
            MserMeasurement msmt = msmtColor.measurement_;
            double majAxisLength = msmt.second.x();
            double minAxisLength = msmt.second.y();
            double theta = msmt.first.theta();
            double cvTheta = msmt.first.theta()*180/M_PI; //opencv wants angles in degrees
            //Draw measurement on image
            cv::Point center = cv::Point(msmt.first.x(),msmt.first.y());
            cv::Size axes = cv::Size(majAxisLength,minAxisLength);
            cv::Point majAxisTip = cv::Point(center.x + majAxisLength*cos(theta),center.y + majAxisLength*sin(theta));
            cv::Point minAxisTip = cv::Point(center.x + minAxisLength*cos(theta + M_PI/2), center.y + minAxisLength*sin(theta + M_PI/2));
            cv::Scalar color = msmtColor.color_;
            int thickness = 5;
            int startAngle = 0;
            int endAngle = 360;//draw entire ellipse
            cv::ellipse(myImage,center,axes,cvTheta,startAngle,endAngle,color,thickness);
            cv::line(myImage,center,majAxisTip,color,thickness);
            cv::line(myImage,center,minAxisTip,color,thickness);
            outputImages.push_back(myImage);
        }
    }
    writeImages(outputImages,"MeasurementsFrame%04i.bmp");
}

void Tracker::writeImages(const std::vector<cv::Mat>& images, const std::string& format) const{ //Print out images to check results.
    //Save tracker images to disk
    int imgDirectorySuccess = mkdir(input_.imagePath().c_str(), 0777);
    for (int f = 0; f < images.size(); f++){
        char imgFileName[200];
        cv::Mat videoFrame = images[f];
        strcpy(imgFileName,input_.imagePath().c_str());
        strcat(imgFileName,format.c_str());
        sprintf(imgFileName, imgFileName,f);
        cv::imwrite(imgFileName, videoFrame);
    }
}


