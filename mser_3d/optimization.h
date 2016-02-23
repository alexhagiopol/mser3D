//
// Created by alex on 2/16/16.
//

#pragma once

Values expressionsOptimization(MserObject& initialGuess, std::vector<MserMeasurement>& measurements, std::vector<SimpleCamera>& cameras, int iterations);
std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(std::vector<MserTrack>& tracks, std::vector<Pose3>& VOposes);



