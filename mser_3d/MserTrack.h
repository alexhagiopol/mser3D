//
// Created by alex on 1/11/16.
//

#ifndef MSER_3D_MSERTRACK_H
#define MSER_3D_MSERTRACK_H

struct MserTrack{
    int colorR; //colors to be assigned to final object
    int colorG;
    int colorB;
    std::vector<int> frameNumbers; //must have same length as measurements
    std::vector<MserMeasurement> measurements; //must have same length as frame numbers
};

#endif //MSER_3D_MSERTRACK_H
