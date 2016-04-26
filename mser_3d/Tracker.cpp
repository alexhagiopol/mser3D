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
    std::cerr << "TRACKER: Read " << images_.size() << " images." << endl;
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

//Single utility function for OpenCV display machinery
void Tracker::display(const cv::Mat& image, const std::string& title, const int& wait) const{
    cv::namedWindow( title, cv::WINDOW_AUTOSIZE );// Create a window for display.
    cv::imshow( title, image );                   // Show our image inside it.
    cv::waitKey(wait);                            // Wait for a keystroke in the window
}

void Tracker::observeMSERs() {
    //Observe MSERs and place results into Frame data structure.
    cerr << "TRACKER: Observing MSERs..." << endl;
    int cameraPoseIndex = 0; //Assumes that first image in images_ matches first pose in input_.VOCameraPoses() TODO: Better alignment of pose data and image data
    std::vector<Pose3> cameraPoses;
    input_.VOCameraPoses(cameraPoses);
    for (int f = 0; f < images_.size() - 1; f++) { //minus 1 because final image tends to be blank
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
        for (int r = 0; r < regions.size(); r++) {
            cv::RotatedRect rr = fitEllipse(regions[r]); //fit ellipse to region. Store info in rotated rectangle data structure
            if (rr.center.x > 0 && rr.center.y > 0){ //See http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=fitellipse#fitellipse for explanation. X and Y center points may be negative in some circumstances.
                Pose2 ellipsePose = Pose2(rr.center.x,rr.center.y,rr.angle*M_PI/180); //convert to radians from OpenCV degrees.
                Point2 ellipseAxes = Point2(rr.size.height,rr.size.width);
                MserMeasurement msmt = MserMeasurement(ellipsePose,ellipseAxes);
                //Compute average color of region; useful in tracking later.
                gtsam::Vector3 colorForCalculations = Z_3x1;
                int numPoints = regions[r].size();
                for (int p = 0; p < numPoints; p++){
                    cv::Vec3b pointColor = image.at<cv::Vec3b>(regions[r][p]);
                    gtsam::Vector3 tempColor(pointColor.val[0],pointColor.val[1],pointColor.val[2]);
                    colorForCalculations = colorForCalculations + tempColor / numPoints;
                }
                cv::Vec3b regionAverageColor(colorForCalculations[0],colorForCalculations[1],colorForCalculations[2]); //use average colors
                MserMeasurementColor msmtColor = MserMeasurementColor(msmt,regionAverageColor);
                measurements.push_back(msmtColor);
                //ellipse(image, rr, cv::Scalar(regionAverageColor.val[0],regionAverageColor.val[1],regionAverageColor.val[2]),4);
            }
        }
        //display(image,"INPUT",1);
        if (f < cameraPoses.size()){ //If we have more images than camera poses, tough luck. We won't make more Frame data structures. See TODO above.
            Frame frame = Frame(f,cameraPoses[f],image,measurements);
            frames_.push_back(frame);
            cameraPoseIndex ++;
        }
    }
    cerr << "TRACKER: Done observing MSERs."  << endl;
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
            cv::Vec3b color = msmtColor.color_;
            cv::Scalar sColor(color.val[0],color.val[1],color.val[2]);
            int thickness = 5;
            int startAngle = 0;
            int endAngle = 360;//draw entire ellipse
            cv::ellipse(myImage,center,axes,cvTheta,startAngle,endAngle,sColor,thickness);
            cv::line(myImage,center,majAxisTip,cv::Scalar(0,0,255),thickness);
            cv::line(myImage,center,minAxisTip,cv::Scalar(255,0,0),thickness);
        }
        outputImages.push_back(myImage);
        display(myImage,"MEASUREMENTS",1);
    }
    writeImages(outputImages,"MeasurementsFrame%04i.bmp");
    cerr << "TEST FRAME OBSERVATIONS: WROTE RESULTS TO " << input_.imagePath() << endl;
}



