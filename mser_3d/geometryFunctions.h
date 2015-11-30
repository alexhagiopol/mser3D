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

void inferObject(std::vector<SimpleCamera>& cameras, std::vector<mserMeasurement>& measurements, mserObject& object){
    //Orientation optimization
    NonlinearFactorGraph cam_pose_graph;
    NonlinearFactorGraph ellipse_pose_graph;
    NonlinearFactorGraph ellipse_dim_graph;
    for (size_t i = 0; i < measurements.size(); i++){
        noiseModel::Diagonal::shared_ptr pose3Noise = noiseModel::Isotropic::Sigma(6,0.1);
        noiseModel::Diagonal::shared_ptr pose2Noise = noiseModel::Isotropic::Sigma(3,0.1);
        noiseModel::Diagonal::shared_ptr point2Noise = noiseModel::Isotropic::Sigma(2,0.1);
        cam_pose_graph.add(PriorFactor<Pose3>(1,cameras[i].pose(), pose3Noise));
        ellipse_pose_graph.add(PriorFactor<Pose2>(1,measurements[i].first,pose2Noise));
        ellipse_dim_graph.add(PriorFactor<Point2>(1,measurements[i].second,point2Noise));
    }
    Values initial_cam_pose, initial_ellipse_pose, initial_ellipse_dim;
    initial_cam_pose.insert(1,cameras[0].pose());
    initial_ellipse_pose.insert(1,measurements[0].first);
    initial_ellipse_dim.insert(1, measurements[0].second);
    Values cam_pose_result = LevenbergMarquardtOptimizer(cam_pose_graph,initial_cam_pose).optimize();
    Values ellipse_pose_result = LevenbergMarquardtOptimizer(ellipse_pose_graph,initial_ellipse_pose).optimize();
    Values ellipse_dim_result = LevenbergMarquardtOptimizer(ellipse_dim_graph,initial_ellipse_dim).optimize();

    cam_pose_result.print("cam pose");
    ellipse_pose_result.print("ellipse pose");
    ellipse_dim_result.print("ellipse dim");
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

std::vector<Point3> convertObjectToObjectPoint3s(mserObject& object, OptionalJacobian<6,8> Dobject = boost::none){
    //Point3 center(object.first.x(),object.first.y(),object.first.z());
    Point3 majAxisTip(object.second.x(),0,0);
    Point3 minAxisTip(0,object.second.y(),0);
    std::vector<Point3> pointsInObjectFrame;
    //pointsInObjectFrame.push_back(center);
    pointsInObjectFrame.push_back(majAxisTip);
    pointsInObjectFrame.push_back(minAxisTip);
    if (Dobject) *Dobject << 0,0,0,0,0,0,1,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,1,
                             0,0,0,0,0,0,0,0;
    return pointsInObjectFrame;
}

std::vector<Point3> convertObjectPoint3sToWorldPoint3s(mserObject& object, std::vector<Point3> pointsInObjectFrame, OptionalJacobian<9,8> Dobject = boost::none, OptionalJacobian<9,6> Dpoints = boost::none) {
    Matrix36 centerDpose, majAxisDpose, minAxisDpose; //Matrices to store results from optional jacobians
    Matrix33 majAxisDpoint, minAxisDpoint; //Matrices to store results from optional jacobians
    Point3 objectCenter = object.first.translation(centerDpose);
    Point3 majAxisInObjectFrame = pointsInObjectFrame[0];
    Point3 minAxisInObjectFrame = pointsInObjectFrame[1];

    Point3 majAxisInWorldFrame = object.first.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    Point3 minAxisInWorldFrame = object.first.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    std::vector<Point3> pointRepresentation;
    pointRepresentation.push_back(objectCenter);
    pointRepresentation.push_back(majAxisInWorldFrame);
    pointRepresentation.push_back(minAxisInWorldFrame);
    //cout << "centerDpose\n" << centerDpose << endl;
    //cout << "majAxisDpose\n" << majAxisDpose << endl;
    //cout << "majAxisDpoint\n" << majAxisDpoint << endl;
    //cout << "minAxisDpose\n" << minAxisDpose << endl;
    //cout << "minAxisDpoint\n" << minAxisDpoint << endl;
    if (Dobject) *Dobject <<   centerDpose(0,0),   centerDpose(0,1),   centerDpose(0,2),   centerDpose(0,3),   centerDpose(0,4),   centerDpose(0,4), 0, 0,
                               centerDpose(1,0),   centerDpose(1,1),   centerDpose(1,2),   centerDpose(1,3),   centerDpose(1,4),   centerDpose(1,4), 0, 0,
                               centerDpose(2,0),   centerDpose(2,1),   centerDpose(2,2),   centerDpose(2,3),   centerDpose(2,4),   centerDpose(2,4), 0, 0,
                              majAxisDpose(0,0),  majAxisDpose(0,1),  majAxisDpose(0,2),  majAxisDpose(0,3),  majAxisDpose(0,4),  majAxisDpose(0,4), 0, 0,
                              majAxisDpose(1,0),  majAxisDpose(1,1),  majAxisDpose(1,2),  majAxisDpose(1,3),  majAxisDpose(1,4),  majAxisDpose(1,4), 0, 0,
                              majAxisDpose(2,0),  majAxisDpose(2,1),  majAxisDpose(2,2),  majAxisDpose(2,3),  majAxisDpose(2,4),  majAxisDpose(2,4), 0, 0,
                              minAxisDpose(0,0),  minAxisDpose(0,1),  minAxisDpose(0,2),  minAxisDpose(0,3),  minAxisDpose(0,4),  minAxisDpose(0,4), 0, 0,
                              minAxisDpose(1,0),  minAxisDpose(1,1),  minAxisDpose(1,2),  minAxisDpose(1,3),  minAxisDpose(1,4),  minAxisDpose(1,4), 0, 0,
                              minAxisDpose(2,0),  minAxisDpose(2,1),  minAxisDpose(2,2),  minAxisDpose(2,3),  minAxisDpose(2,4),  minAxisDpose(2,4), 0, 0;
    if (Dpoints) *Dpoints <<                  0,                  0,                  0,                  0,                  0,                  0,
                                              0,                  0,                  0,                  0,                  0,                  0,
                                              0,                  0,                  0,                  0,                  0,                  0,
                             majAxisDpoint(0,0), majAxisDpoint(0,1), majAxisDpoint(0,2),                  0,                  0,                  0,
                             majAxisDpoint(1,0), majAxisDpoint(1,1), majAxisDpoint(1,2),                  0,                  0,                  0,
                             majAxisDpoint(2,0), majAxisDpoint(2,1), majAxisDpoint(2,2),                  0,                  0,                  0,
                                              0,                  0,                  0, minAxisDpoint(0,0), minAxisDpoint(0,1), minAxisDpoint(0,2),
                                              0,                  0,                  0, minAxisDpoint(1,0), minAxisDpoint(1,1), minAxisDpoint(1,2),
                                              0,                  0,                  0, minAxisDpoint(2,0), minAxisDpoint(2,1), minAxisDpoint(2,2);

    return pointRepresentation;
}

mserMeasurement convertPoint3sToMeasurement(SimpleCamera& camera, std::vector<Point3>& points, OptionalJacobian<5,5> Dcal = boost::none, OptionalJacobian<5,6> Dpose = boost::none, OptionalJacobian<5,9> Dpoints = boost::none){
    Point3 objectCenter = points[0];
    Point3 majorAxisTip = points[1];
    Point3 minorAxisTip = points[2];
    Matrix26 centerDpose, majorDpose, minorDpose;
    Matrix23 centerDpoint, majorDpoint, minorDpoint;
    Matrix25 centerDcal, majorDcal, minorDcal;
    Point2 projectedObjectCenter = camera.project(objectCenter,centerDpose, centerDpoint, centerDcal);
    Point2 projectedMajorAxisTip = camera.project(majorAxisTip, majorDpose, majorDpoint, majorDcal);
    Point2 projectedMinorAxisTip = camera.project(minorAxisTip, minorDpose, minorDpoint, minorDcal);
    Matrix12 oriCtrH, oriMaAxH;
    double orientation = ellipse2DOrientation(projectedObjectCenter, projectedMajorAxisTip,oriCtrH, oriMaAxH);
    Matrix12 majordistanceDmajoraxis, majordistanceDcenter, minordistanceDminoraxis, minordistanceDcenter;
    double majorAxis = projectedMajorAxisTip.distance(projectedObjectCenter,majordistanceDmajoraxis,majordistanceDcenter);
    double minorAxis = projectedMinorAxisTip.distance(projectedObjectCenter,minordistanceDminoraxis,minordistanceDcenter);
    Point2 axes(majorAxis,minorAxis);
    Pose2 pose(projectedObjectCenter.x(),projectedObjectCenter.y(),orientation);
    mserMeasurement measurement(pose,axes);
    return measurement;
}


/*
std::vector<Point3> convertObjectToPoint3s(mserObject& object, OptionalJacobian<9,8> H = boost::none){
    Matrix36 centerDpose, majAxisDpose, minAxisDpose;
    Matrix33 majAxisDpoint, minAxisDpoint;
    Point3 objectCenter = object.first.translation(centerDpose);
    Point3 majAxisInObjectFrame(object.second.x(),0,0);
    Point3 minAxisInObjectFrame(0,object.second.y(),0);



    Point3 majAxisTip = object.first.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    Point3 minAxisTip = object.first.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    std::vector<Point3> pointRepresentation;
    pointRepresentation.push_back(objectCenter);
    pointRepresentation.push_back(majAxisTip);
    pointRepresentation.push_back(minAxisTip);
    cout << "centerDpose\n" << centerDpose << endl;
    cout << "majAxisDpose\n" << majAxisDpose << endl;
    cout << "majAxisDpoint\n" << majAxisDpoint << endl;
    cout << "minAxisDpose\n" << minAxisDpose << endl;
    cout << "minAxisDpoint\n" << minAxisDpoint << endl;
    //if (H) *H << ;
    //TODO: Jacobian from above matrices.
    return pointRepresentation;
}
 */

/*
mserMeasurement mserMeasurementFunction(SimpleCamera& camera, mserObject& object, OptionalJacobian<5,11> H1 = boost::none, OptionalJacobian<5, 8> H2 = boost::none){
//Dimension of H1 Jacobian is 5x11 because mserMeasurement has dimension 5 while a camera has dimension 11 (5 calibration parameters + 6 pose parameters)

}
*/




/*
Values naiveMSEROptimizationExampleExpressions(){
    int numCams = 20;
    double radius = 10.0;
    Point3 target = Point3(0.0,0.0,0.0);
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    std::vector<mserMeasurement> measurements;
    int success = produceMSERMeasurements(cameras, target, measurements);
    typedef Expression<mserObject> mserObject_;
    typedef Expression<mserMeasurement> mserMeasurement_;
    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();
    vector<mserObject>
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
    //measurement noise
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
    vector<Point3> points = createPoints();
    ExpressionFactorGraph graph;
    //pose noise
    Vector6 sigmas; sigmas << Vector3(0.3,0.3,0.3), Vector3(0.1,0.1,0.1);
    Diagonal::shared_ptr poseNoise = Diagonal::Sigmas(sigmas);
    mserObject_ x0('x',0);
    graph.addExpressionFactor(x0, poses[0], poseNoise);
}
*/
#endif //MSER_3D_GEOMETRYFUNCTIONS_H
