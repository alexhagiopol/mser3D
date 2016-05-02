//
// Created by alex on 4/22/16.
//

#pragma once
#include "InputManager.h"
#include "Frame.h"

class Tracker {
public:
    Tracker(InputManager& input);
    void testFrameObservations();
    void readImages();
    void writeImages(const std::vector<cv::Mat>& images, const std::string& format) const;
    void display(const cv::Mat& image, const std::string& title, const int& wait) const;
    void observeMSERs();
    void displayTracks(const std::vector<cv::Mat>& images,const std::vector<MserTrack>& tracks);
    InputManager input_;
    std::vector<Frame> frames_;
    std::vector<cv::Mat> images_;
};

