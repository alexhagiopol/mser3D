//
// Created by alex on 4/22/16.
//

#pragma once
#include "InputManager.h"
#include "Frame.h"

class Tracker {
public:
    Tracker(InputManager& input);
    void writeImages(std::vector<cv::Mat>& images) const;
private:
    void readImages();
    void observeMSERs();
    InputManager input_;
    std::vector<Frame> frames_;
    std::vector<cv::Mat> images_;
};

