//
// Created by alex on 4/24/16.
//

#pragma once
#include "MserMeasurement.h"
#include <opencv2/core/core.hpp>
using namespace gtsam;

struct MserMeasurementColor {
    MserMeasurement measurement_;
    cv::Scalar color_; //BGR format from OpenCV
    MserMeasurementColor(const MserMeasurement& measurement, const cv::Scalar& color) : measurement_(measurement), color_(color){}
};

