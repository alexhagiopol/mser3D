//
// Created by alex on 4/24/16.
//

#pragma once
#include "MserMeasurementColor.h"
#include <gtsam/geometry/Pose3.h>
#include <opencv2/core/core.hpp>

using namespace gtsam;

class Frame {
public:
    int id_;
    Pose3 cameraPose_;
    cv::Mat image_;
    std::vector<MserMeasurementColor> measurements;
};

