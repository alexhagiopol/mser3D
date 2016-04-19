//
// Created by alex on 2/16/16.
//
#pragma once
#include "MserMeasurement.h"
#include "MserObject.h"
#include "MserTrack.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

typedef std::vector<double> record_t;
typedef std::vector<record_t> data_t;

std::istream& operator >> (std::istream& ins, record_t& record);
std::istream& operator >> (std::istream& ins, data_t& data);

class InputManager {
public:
    InputManager(std::string settingsPath);
    bool successfulInput() { return successfulInput_;}
    std::string videoPath(){ return videoPath_;}
    std::string BALPath() { return BALPath_;}
    std::string CSVPath() { return CSVPath_;}
    std::string VertexShaderPath() {return VertexShaderPath_;}
    std::string FragmentShaderPath() {return FragmentShaderPath_;}
    double cameraFx() const { return cameraFx_;}
    double cameraFy() const { return cameraFy_;}
    double cameraS() const { return cameraS_;}
    double cameraCx() const { return cameraCx_;}
    double cameraCy() const { return cameraCy_;}
    double minDiversity() const { return minDiversity_;}
    double minArea() const { return minArea_;}
    double maxArea() const { return maxArea_;}
    bool showRays() const {return showRays_;}
    void getMSERMeasurementTracks(std::vector<gtsam::MserTrack>& tracks) const {tracks = MSERMeasurementTracks_;}
    void getVOCameraPoses(std::vector<gtsam::Pose3>& poses) const {poses = VOCameraPoses_;}
private:
    bool successfulInput_;
    //File paths
    std::string videoPath_;
    std::string BALPath_;
    std::string CSVPath_;
    std::string VertexShaderPath_;
    std::string FragmentShaderPath_;
    //Camera parameters
    double cameraFx_;
    double cameraFy_;
    double cameraS_;
    double cameraCx_;
    double cameraCy_;
    //MSER parameters
    double minDiversity_;
    double minArea_;
    double maxArea_;
    //Visualization settings
    bool showRays_;
    //Data structures for MSER tracks and camera poses
    std::vector<gtsam::MserTrack> MSERMeasurementTracks_;
    std::vector<gtsam::Pose3> VOCameraPoses_;
    //Functions for getting tracks and poses
    std::vector<gtsam::MserTrack> getMserTracksFromCSV(std::string csvFile);
    std::vector<gtsam::Pose3> getPosesFromBAL(std::string balFile);
};
