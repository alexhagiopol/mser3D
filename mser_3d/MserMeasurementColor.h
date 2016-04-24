//
// Created by alex on 4/24/16.
//

#pragma once
#include "MserMeasurement.h"
using namespace gtsam;

class MserMeasurementColor {
private:
    MserMeasurement measurement_;
    Vector3 color_; //RGB format
public:
    MserMeasurementColor(const MserMeasurement& measurement, const Vector3& color) : measurement_(measurement), color_(color){}
    MserMeasurement measurement(){return measurement_;}
    Vector3 color() {return color_;} //RGB format
};

