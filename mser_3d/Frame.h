//
// Created by alex on 4/24/16.
//

#pragma once
#include "MserMeasurementColor.h"
#include <gtsam/geometry/Pose3.h>
#include <opencv2/core/core.hpp>

using namespace gtsam;

struct Frame {
    int id_;
    Pose3 cameraPose_;
    cv::Mat image_;
    std::vector<MserMeasurementColor> measurementsColor_;
    Frame(const int& id, const Pose3& cameraPose, const cv::Mat& image, const std::vector<MserMeasurementColor>& measurementsColor) : id_(id), cameraPose_(cameraPose), image_(image), measurementsColor_(measurementsColor) {}
};

