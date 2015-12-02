//
// Created by Alex Hagiopol on 11/30/15.
//

#ifndef MSER_3D_MEASUREMENTFUNCTION_H
#define MSER_3D_MEASUREMENTFUNCTION_H

#include "geometryFunctions.h"
#include "mserClasses.h"
#include "boost/optional.hpp"
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <math.h>
#include <string>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;

struct pointsPose{ //note: Point3s are in *object frame*
    Point3 majAxisTip;
    Point3 minAxisTip;
    Pose3 objectPose;
};

pointsPose convertObjectToObjectPointsPose(const mserObject& object, OptionalJacobian<12,8> Dobject = boost::none){
    pointsPose myPointsPose;
    myPointsPose.majAxisTip = Point3(object.second.x(),0,0);
    myPointsPose.minAxisTip = Point3(0,object.second.y(),0);
    myPointsPose.objectPose = object.first;
    if (Dobject) *Dobject << 0,0,0,0,0,0,1,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,1,
                             0,0,0,0,0,0,0,0,
                             1,0,0,0,0,0,0,0,
                             0,1,0,0,0,0,0,0,
                             0,0,1,0,0,0,0,0,
                             0,0,0,1,0,0,0,0,
                             0,0,0,0,1,0,0,0,
                             0,0,0,0,0,1,0,0;
    return myPointsPose;
}

std::vector<Point3> convertObjectPointsPoseToWorldPoint3s(pointsPose& objectPointsPose, OptionalJacobian<9,12> Dpointspose = boost::none){
    Matrix36 centerDpose, majAxisDpose, minAxisDpose; //Matrices to store results from optional jacobians
    Matrix33 majAxisDpoint, minAxisDpoint; //Matrices to store results from optional jacobians
    Point3 objectCenter = objectPointsPose.objectPose.translation(centerDpose);
    Point3 majAxisInObjectFrame = objectPointsPose.majAxisTip;
    Point3 minAxisInObjectFrame = objectPointsPose.minAxisTip;

    Point3 majAxisInWorldFrame = objectPointsPose.objectPose.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    Point3 minAxisInWorldFrame = objectPointsPose.objectPose.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    std::vector<Point3> pointRepresentation;
    pointRepresentation.push_back(objectCenter);
    pointRepresentation.push_back(majAxisInWorldFrame);
    pointRepresentation.push_back(minAxisInWorldFrame);
    if (Dpointspose){
        Eigen::MatrixXd zeros33 = Eigen::MatrixXd::Zero(3,3);
        Eigen::MatrixXd topLeft = Eigen::MatrixXd::Zero(3,6);
        Eigen::MatrixXd middleLeft(3,6);
        Eigen::MatrixXd bottomLeft(3,6);
        Eigen::MatrixXd left(9,6);
        Eigen::MatrixXd right(9,6);
        Eigen::MatrixXd Dpointspose_(9,12);
        middleLeft << majAxisDpoint, zeros33;
        bottomLeft << zeros33, minAxisDpoint;
        left << topLeft,
                middleLeft,
                bottomLeft;
        right << centerDpose,
                 majAxisDpose,
                 minAxisDpose;
        Dpointspose_ << left, right;
        *Dpointspose << Dpointspose_;
    }
    return pointRepresentation;
}

std::vector<Point2> convertWorldPoint3sToCameraPoint2s(const SimpleCamera& camera, std::vector<Point3>& points, OptionalJacobian<6,6> Dpose = boost::none, OptionalJacobian<6,5> Dcal = boost::none, OptionalJacobian<6,9> Dpoints = boost::none){
    Point3 objectCenter = points[0];
    Point3 majorAxisTip = points[1];
    Point3 minorAxisTip = points[2];
    Matrix26 centerDpose, majorDpose, minorDpose;
    Matrix23 centerDpoint, majorDpoint, minorDpoint;
    Matrix25 centerDcal, majorDcal, minorDcal;
    Point2 projectedObjectCenter = camera.project(objectCenter, centerDpose, centerDpoint, centerDcal);
    Point2 projectedMajorAxisTip = camera.project(majorAxisTip, majorDpose, majorDpoint, majorDcal);
    Point2 projectedMinorAxisTip = camera.project(minorAxisTip, minorDpose, minorDpoint, minorDcal);
    std::vector<Point2> cameraPoint2s;
    cameraPoint2s.push_back(projectedObjectCenter);
    cameraPoint2s.push_back(projectedMajorAxisTip);
    cameraPoint2s.push_back(projectedMinorAxisTip);

    if (Dpose) {
        Eigen::MatrixXd Dpose_(6,6);
        Dpose_ << centerDpose,
                majorDpose,
                minorDpose;
        *Dpose << Dpose_;
    }
    if (Dcal){
        Eigen::MatrixXd Dcal_(6,5);
        Dcal_ << centerDcal,
                 majorDcal,
                 minorDcal;
        *Dcal <<  Dcal_;
    }
    if (Dpoints){
        Eigen::MatrixXd Dpoints_(6,9);
        Eigen::MatrixXd zeros23 = Eigen::MatrixXd::Zero(2,3);
        Eigen::MatrixXd top(2,9);
        Eigen::MatrixXd middle(2,9);
        Eigen::MatrixXd bottom(2,9);
        top << centerDpoint,zeros23, zeros23;
        middle << zeros23,majorDpoint,zeros23;
        bottom << zeros23, zeros23, minorDpoint;
        Dpoints_ << top,
                    middle,
                    bottom;
        *Dpoints << Dpoints_;
    }
    return cameraPoint2s;
}

mserMeasurement convertCameraPoint2sToMeasurement(std::vector<Point2>& cameraPoint2s, OptionalJacobian<5,6> Dpoints = boost::none){
    Point2 measurementCenter = cameraPoint2s[0]; //Jacobian for this is 2x2 identity matrix
    Matrix12 thetaDcenter, thetaDmajor, A1Dmajor, A1Dcenter, A2Dminor, A2Dcenter;
    double theta = ellipse2DOrientation(measurementCenter, cameraPoint2s[1], thetaDcenter, thetaDmajor);
    double A1 = cameraPoint2s[1].distance(measurementCenter, A1Dmajor, A1Dcenter);
    double A2 = cameraPoint2s[2].distance(measurementCenter, A2Dminor, A2Dcenter);
    Pose2 ellipsePose(measurementCenter.x(),measurementCenter.y(),theta);
    Point2 axisLengths(A1,A2);
    mserMeasurement measurement(ellipsePose,axisLengths);
    if (Dpoints){
        *Dpoints <<                 1,                 0,                0,                0,             0,             0,
                                    0,                 1,                0,                0,             0,             0,
                    thetaDcenter(0,0), thetaDcenter(0,1), thetaDmajor(0,0), thetaDmajor(0,1),             0,             0,
                       A1Dcenter(0,0),    A1Dcenter(0,1),    A1Dmajor(0,0),    A1Dmajor(0,1),             0,             0,
                       A2Dcenter(0,0),    A2Dcenter(0,1),                0,                0, A2Dminor(0,0), A2Dminor(0,1);
    }
    return measurement;
}

mserMeasurement measurementFunction(const SimpleCamera& camera, const mserObject& object, OptionalJacobian<5,11> Dcamera = boost::none, OptionalJacobian<5,8> Dobject = boost::none){
    //Part 1: object -> Point3s in object Frame
    Eigen::MatrixXd objectpointsposeDobject12_8(12,8);
    pointsPose objectPointsPose = convertObjectToObjectPointsPose(object, objectpointsposeDobject12_8);

    //Part 2: Point3s in object frame -> Point3s in world frame
    Eigen::MatrixXd worldpointsDobjectpointspose9_12(9,12);
    std::vector<Point3> worldPoints = convertObjectPointsPoseToWorldPoint3s(objectPointsPose, worldpointsDobjectpointspose9_12);

    //Part 3: Point3s in world frame -> Point2s in camera frame
    Matrix66 campointsDpose66;
    Matrix65 campointsDcal65;
    Matrix69 campointsDworldpoints69;
    std::vector<Point2> cameraPoints = convertWorldPoint3sToCameraPoint2s(camera, worldPoints, campointsDpose66, campointsDcal65, campointsDworldpoints69);

    //Part 4: Point2s in camera frame -> measurement
    Matrix56 msmtDcampoints56;
    mserMeasurement measurement = convertCameraPoint2sToMeasurement(cameraPoints, msmtDcampoints56);

    //Provide Jacobians
    //if (Dpose) *Dpose << msmtDcampoints56*campointsDpose66;
    //if (Dcal) *Dcal << msmtDcampoints56*campointsDcal65;
    if (Dcamera) {
        Eigen::MatrixXd Dcamera_(5,11);
        Dcamera_ << msmtDcampoints56*campointsDpose66, msmtDcampoints56*campointsDcal65;
        *Dcamera << Dcamera_;
    }
    if (Dobject) *Dobject << msmtDcampoints56*campointsDworldpoints69*worldpointsDobjectpointspose9_12*objectpointsposeDobject12_8;

    /*
    //Print out Jacobians for debugging
    cout << "objectpointsposeDobject12_8\n" << objectpointsposeDobject12_8 << endl;
    cout << "worldpointsDobjectpointspose9_12\n" << worldpointsDobjectpointspose9_12 << endl;
    cout << "campointsDpose66\n" << campointsDpose66 << endl;
    cout << "campointsDcal65\n" << campointsDcal65 << endl;
    cout << "campointsDworldpoints69\n" << campointsDworldpoints69 << endl;
    cout << "msmtDcampoints56\n" << msmtDcampoints56 << endl;
    cout << "Dpose\n" << *Dpose << endl;
    cout << "Dcal\n" << *Dcal << endl;
    cout << "Dobject\n" << *Dobject << endl;
    */
    return measurement;
}

typedef Expression<mserObject> mserObject_;
typedef Expression<mserMeasurement> mserMeasurement_;
typedef Expression<SimpleCamera> SimpleCamera_;

inline mserMeasurement_ measurementFunctionExp(const SimpleCamera_ &camera_, const mserObject_ &object_) {
    mserMeasurement (*f)(const SimpleCamera&, const mserObject&, OptionalJacobian<5,11>, OptionalJacobian<5,8>) = &measurementFunction;
    return mserMeasurement_(f, camera_, object_);
    //return mserMeasurement_(&measurementFunction, camera_, object_, OptionalJacobian<5, 11>, OptionalJacobian<5, 8>);
}

std::vector<mserMeasurement> createIdealMeasurements(std::vector<SimpleCamera>& cameras, mserObject& object){
    std::vector<mserMeasurement> measurements;
    for (size_t i = 0; i < cameras.size(); i++){
        mserMeasurement measurement = measurementFunction(cameras[i], object);
        measurements.push_back(measurement);
    }
    return measurements;
}

Values expressionsOptimization(mserObject& object){
    Cal3_S2 K(500.0, 500.0, 0.1, 640/2, 480/2); //camera parameters
    Isotropic::shared_ptr measurementNoise = Isotropic::Sigma(5, 1.0); // one pixel in every dimension

    //Ground truth object is passed to this function. Create vectors with measurements, cameras, and camera poses.
    std::vector<SimpleCamera> cameras = alexCreateCameras(20,object.first.translation(),20); //make a bunch of cameras to pass to measurement function
    std::vector<mserMeasurement> measurements = createIdealMeasurements(cameras, object); //synthetic measurements directly from measurement function
    std::vector<Pose3> poses;
    for (size_t i = 0; i < cameras.size(); i++){
        poses.push_back(cameras[i].pose());
    }

    // Create a factor graph
    ExpressionFactorGraph graph;

    // Specify uncertainty on first pose prior. Same as example.
    Vector6 sigmas; sigmas << Vector3(0.3,0.3,0.3), Vector3(0.1,0.1,0.1);
    Diagonal::shared_ptr poseNoise = Diagonal::Sigmas(sigmas);

    Pose3_ x0('x',0);
    graph.addExpressionFactor(x0, poses[0], poseNoise);

    // We create a constant Expression for the calibration here
    Cal3_S2_ cK(K);

    for (size_t i = 0; i < poses.size(); ++i) {
        const SimpleCamera_ c(cameras[i]); //expression for the camera created here
        for (size_t j = 0; j < 1; j++){ //I know this is not needed but I want to look *exactly* like the example. We only have 1 measurement per camera pose.
            mserMeasurement measurement = measurements[i];
            // Below an expression for the prediction of the measurement:
            mserObject_ o('o',0);
            mserMeasurement_ prediction = measurementFunctionExp(c,o);
            graph.addExpressionFactor(prediction,measurement,measurementNoise);
        }
    }
    // Add prior on first point to constrain scale, again with ExpressionFactor
    Isotropic::shared_ptr objectNoise = Isotropic::Sigma(8, 0.1);
    graph.addExpressionFactor(mserObject_('o', 0), object, objectNoise);

    // Create perturbed initial
    Values initial;
    Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));
    for (size_t i = 0; i < poses.size(); ++i){
        initial.insert(Symbol('x', i), poses[i].compose(delta));
    }
    for (size_t j = 0; j < 1; j++){
        initial.insert(Symbol('o', 0), object);
    }

    cout << "initial error = " << graph.error(initial) << endl;
    Values result = DoglegOptimizer(graph, initial).optimize();
    cout << "final error = " << graph.error(result) << endl;
    return result;
}

#endif //MSER_3D_MEASUREMENTFUNCTION_H
