//
// Created by Alex Hagiopol on 10/26/15.
//

#ifndef MSER_3D_ALEXFUNCTIONS_H
#define MSER_3D_ALEXFUNCTIONS_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <string>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;
std::vector<Pose3> alexCreatePoses(double radius, Point3 target, int numPoses){
    std::vector<gtsam::Pose3> poses;
    double theta = 0.0;
    Point3 up = Point3(0,0,1);
    for(int i = 0; i < numPoses; i++){
        Point3 position = Point3(target.x() + radius*cos(theta), target.y() + radius*sin(theta), target.z() + 0.0);
        SimpleCamera tempCam = SimpleCamera::Lookat(position, target, up);
        theta += 2*M_PI/numPoses;
        poses.push_back(tempCam.pose());
    }
    return poses;
}

std::vector<SimpleCamera> alexCreateCameras(double radius, Point3 target, int numCams){
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)); //made up calibration for now; replace with Jing's calibration later
    std::vector<SimpleCamera> cameras;
    double theta = 0.0;
    Point3 up = Point3(0,0,1);
    for(int i = 0; i < numCams; i++){
        Point3 position = Point3(target.x() + radius*cos(theta), target.y() + radius*sin(theta), target.z() + 0.0);
        SimpleCamera tempCam = SimpleCamera::Lookat(position, target, up, *K);
        cameras.push_back(tempCam);
        theta += 2*M_PI/numCams;
    }
    return cameras;
}

void printJingData(){
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
                    const Pose3 pose = camera.pose();
                    pose.print("Camera pose:\n");
                }
}

/* ************************************************************************* */
//Compatible with GTSAM example code. Prefer using this over including Dataset.h
std::vector<gtsam::Point3> createPoints() {

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.push_back(gtsam::Point3(10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,-10.0));
    return points;
}

/* ************************************************************************* */
//Compatible with GTSAM example code. Prefer using this over including Dataset.h
std::vector<gtsam::Pose3> createPoses() {
    // Create the set of ground-truth poses
    std::vector<gtsam::Pose3> poses;
    double radius = 30.0;
    int i = 0;
    double theta = 0.0;
    gtsam::Point3 up(0,0,1);
    gtsam::Point3 target(0,0,0);
    for(; i < 8; ++i, theta += 2*M_PI/8) {
        gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
        poses.push_back(camera.pose());
    }
    return poses;
}

#endif //MSER_3D_ALEXFUNCTIONS_H
