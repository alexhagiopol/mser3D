//
// Created by alex on 2/16/16.
//

#pragma once

Values expressionsOptimization(const MserObject& initialGuess, const std::vector<MserMeasurement>& measurements, const std::vector<SimpleCamera>& cameras, int iterations);
std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(const std::vector<MserTrack>& tracks, const std::vector<Pose3>& VOposes, bool showEachStep, int levMarIterations);
std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurementsViaTriangulation(const std::vector<MserTrack>& tracks, const std::vector<Pose3>& VOposes);


