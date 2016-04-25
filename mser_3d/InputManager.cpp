//
// Created by alex on 2/16/16.
//

#include "InputManager.h"
#include <opencv2/core/core.hpp> //need for reading YAML files
#include <gtsam/slam/dataset.h>
#include <opencv2/highgui/highgui.hpp>
#include "boost/optional.hpp"

using namespace gtsam;

std::istream& operator >> (std::istream& ins, record_t& record){
     record.clear();
     std::string line;
     std::getline(ins, line);
     std::stringstream ss(line);
     std::string field;
     while (std::getline(ss, field, ',')){
          std::stringstream fs(field);
          double f = 0.0;
          fs >> f;
          record.push_back(f);
     }
     return ins;
}

std::istream& operator >> (std::istream& ins, data_t& data){
     data.clear();
     record_t record;
     while(ins >> record){
          data.push_back(record);
     }
     return ins;
}

void InputManager::processSettingsFile(const std::string& settingsPath){
     SettingsPath_ = settingsPath;
     cv::FileStorage settings(SettingsPath_.c_str(), cv::FileStorage::READ);
     if (!settings.isOpened()){
          std::cerr << std::endl << "INPUT MANAGER: Wrong path to settings! Was given " << SettingsPath_ << std::endl;
          std::cerr << "INPUT MANAGER: Fix settings file location and re-execute!" << std::endl;
          successfulInput_ = false;
     } else {
          std::cerr << "INPUT MANAGER: Reading settings from " << SettingsPath_ << std::endl;
          //Paths
          VideoPath_ = (std::string) settings["VideoPath"];
          BALPath_ = (std::string) settings["BALPath"];
          CSVPath_ = (std::string) settings["CSVPath"];
          VertexShaderPath_ = (std::string) settings["VertexShaderPath"];
          FragmentShaderPath_ = (std::string) settings["FragmentShaderPath"];
          ImagePath_ = (std::string) settings["ImagePath"];
          //Camera settings
          cameraFx_ = settings["Camera.fx"];
          cameraFy_ = settings["Camera.fy"];
          cameraS_ = settings["Camera.fy"];
          cameraCx_ = settings["Camera.cx"];
          cameraCy_ = settings["Camera.cy"];
          //MSER Settings
          delta_ = settings["MSER.Delta"];
          minArea_ = settings["MSER.MinArea"];
          maxArea_ = settings["MSER.MaxArea"];
          maxVariation_ = settings["MSER.MaxVariation"];
          minDiversity_ = settings["MSER.MinDiversity"];
          maxEvolution_ = settings["MSER.MaxEvolution"];
          areaThreshold_ = settings["MSER.AreaThreshold"];
          minMargin_ = settings["MSER.MinMargin"];
          edgeBlurSize_ = settings["MSER.EdgeBlurSize"];
          //Options
          int intShowRays = settings["Vis.ShowRays"];
          showRays_ = (bool) intShowRays;
          successfulInput_ = true;
          //Call functions to grab data
          getMSERMeasurementTracks();
          getVOCameraPoses();
     }
}

void InputManager::getVOCameraPoses(){
     std::cerr << "INPUT MANAGER: Reading " << BALPath_ << std::endl;
     SfM_data mydata;
     readBAL(BALPath_, mydata);
     BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
                         const Pose3 pose = camera.pose();
                         VOCameraPoses_.push_back(pose);
                    }
     std::cerr << "INPUT MANAGER: Found " << VOCameraPoses_.size() << " VO poses." << std::endl;
}

void InputManager::getMSERMeasurementTracks(){
     std::cerr << "INPUT MANAGER: Reading " << CSVPath_ << std::endl;
     std::ifstream infile(CSVPath_);
     data_t data;
     infile >> data;
     infile.close();
     for (int r = 1; r < data.size(); r++){ //start at row 1 because row 0 does not contain data
          MserTrack track;
          int numMSERS = data[r][1];
          for (int c = 2; c < 2 + 6*numMSERS; c = c + 6){
               int frameNum = (int) data[r][c];
               double x = data[r][c+1];
               double y = data[r][c+2];
               double a = data[r][c+3];
               double b = data[r][c+4];
               double theta = data[r][c+5];
               int end = data[r].size() - 1;
               int R = data[r][end - 2];
               int G = data[r][end - 1];
               int B = data[r][end];
               Pose2 pose(x,y,theta);
               Point2 axes(a,b);
               MserMeasurement measurement(pose,axes);
               track.frameNumbers.push_back(frameNum);
               track.measurements.push_back(measurement);
               track.colorR = R;
               track.colorG = G;
               track.colorB = B;
          }
          MSERMeasurementTracks_.push_back(track);
     }
     std::cerr << "INPUT MANAGER: Found " << MSERMeasurementTracks_.size() << " MSER tracks." << std::endl;
}



