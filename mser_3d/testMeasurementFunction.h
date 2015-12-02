//
// Created by Alex Hagiopol on 11/30/15.
//

#ifndef MSER_3D_TESTMEASUREMENTFUNCTION_H
#define MSER_3D_TESTMEASUREMENTFUNCTION_H

#include "measurementFunction.h"
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

void testExpressionsOptimization(){
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject groundTruthObject(objectPose,objectAxes); //ground truth object
    Values correct;
    correct.insert(Symbol('o',0),groundTruthObject);
    Values result = expressionsOptimization(groundTruthObject);
    if (correct.equals(result,0.001)){
        cout << "EXPRESSIONS OPTIMIZATION PASSED" << endl;
    } else {
        cout << "EXPRESSIONS OPTIMIZATION FAILED" << endl;
    }
}

void testMeasurementFunction(){
    //object parameters
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject object(objectPose,objectAxes);
    Point3 majorAxisTip(3,0,0);
    Point3 minorAxisTip(0,1,0);
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, objectCenter, up, *K);
    Eigen::MatrixXd Dcamera(5,11);
    Matrix58 Dobject;
    mserMeasurement returnedMeasurement = measurementFunction(camera, object, Dcamera, Dobject);
    mserMeasurement correctMeasurement(Pose2(320,240,0),Point2(150,50)); //hand calculated measurement


    if (gtsam::traits<mserMeasurement>::Equals(correctMeasurement,returnedMeasurement,0.001)){
        cout << "MEASUREMENT FUNCTION TEST #1 PASSED" << endl;
    } else {
        cout << "MEASUREMENT FUNCTION TEST #1 FAILED" << endl;
        gtsam::traits<mserMeasurement>::Print(correctMeasurement, "CORRECT");
        gtsam::traits<mserMeasurement>::Print(returnedMeasurement, "RETURNED");
    }
    /*
    if (mserMeasurement::Equals(correctMeasurement,returnedMeasurement,0.001)){
        cout << "MEASUREMENT FUNCTION TEST #1 PASSED" << endl;
    } else {
        cout << "MEASUREMENT FUNCTION TEST #1 FAILED" << endl;
        mserMeasurement::Print(correctMeasurement, "CORRECT");
        mserMeasurement::Print(returnedMeasurement, "RETURNED");
    }
     */
}

void testAllMeasurementFunction(){
    testMeasurementFunction();
    testExpressionsOptimization();
}

#endif //MSER_3D_TESTMEASUREMENTFUNCTION_H
