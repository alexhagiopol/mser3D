//
// Created by alex on 11/9/15.
//

#ifndef MSER_3D_GEOMETRYFUNCTIONS_H
#define MSER_3D_GEOMETRYFUNCTIONS_H

#include "mserClasses.h"
#include "boost/optional.hpp"
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>  //required for optimization with Expressions syntax
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <math.h>
#include <string>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;
using namespace noiseModel;

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
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 360/2)); //made up calibration for now; replace with Jing's calibration later
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

//Example optimization using a new object that consists of a pair of Point2 objects.
//Essentially computes the average of a set of these objects to serve as unit test.
//Uses standard GTSAM syntax i.e. no Expressions yet.
Values pointPairOptimize(){
    MyPoint2Pair p1(Point2(1,2),Point2(3,4));
    MyPoint2Pair p2(Point2(10,20),Point2(30,40));
    NonlinearFactorGraph graph;
    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(4, 0.1);
    graph.add(PriorFactor<MyPoint2Pair>(1,p1,pointNoise));
    graph.add(PriorFactor<MyPoint2Pair>(1,p2,pointNoise));
    Values initial;
    initial.insert(1, p1);
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print();
    return result;
}

//Example optimization using 3D points. Given a set of data points
Values locateObject(Point3 target, Point3 guess, int numCams, double radius){
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    //produceMSERMeasurements(cameras); //virtual stuff in opengl
    NonlinearFactorGraph graph;
    for (int i = 0; i < numCams; i++){
        Point2 measurement = cameras[i].project(target);
        noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
        Point3 backProjectedPoint3 = cameras[i].backproject(measurement, radius); //assume depth is known from other methods e.g. VO
        graph.add(PriorFactor<Point3>(1, backProjectedPoint3, pointNoise));
    }
    //estimate object centroid location
    Values initial;
    initial.insert(1, guess);
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    //result.print();
    return result;
}

//Returns orientation of 2D ellipse given center point and major axis point
double ellipse2DOrientation(Point2& center, Point2& majorAxisPoint, OptionalJacobian<1,2> H1 = boost::none, OptionalJacobian<1,2> H2 = boost::none){
    //Math reference: https://en.wikipedia.org/wiki/Atan2
    //C++ atan2(y,x) reference: http://en.cppreference.com/w/c/numeric/math/atan2
    double y = majorAxisPoint.y() - center.y();
    double x = majorAxisPoint.x() - center.x();
    double orientation = atan2(y,x);
    if (H1) *H1 << y/(x*x + y*y), -1*x/(x*x + y*y); //derivative wrt center point
    if (H2) *H2 << -1*y/(x*x + y*y), x/(x*x + y*y);//derivative wrt axis point
    return orientation;
}

#endif //MSER_3D_GEOMETRYFUNCTIONS_H
