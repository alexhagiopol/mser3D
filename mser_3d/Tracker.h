//
// Created by alex on 4/22/16.
//

#pragma once
#include "InputManager.h"
#include "Frame.h"

class Tracker {
public:
    Tracker(InputManager& input);
    void writeImages(const std::vector<cv::Mat>& images, const std::string& format) const;
    void testFrameObservations();
private:
    void readImages();
    void observeMSERs();
    InputManager input_;
    std::vector<Frame> frames_;
    std::vector<cv::Mat> images_;
};

