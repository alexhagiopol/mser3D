//
// Created by alex on 1/11/16.
//
#pragma once
#include "MserMeasurement.h"
namespace gtsam{
    struct MserTrack{
        int colorR; //colors to be assigned to final object
        int colorG;
        int colorB;
        std::vector<int> frameNumbers; //must have same length as measurements
        std::vector<MserMeasurement> measurements; //must have same length as frame numbers
    };
}

