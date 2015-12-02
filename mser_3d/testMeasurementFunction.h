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
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject groundTruthObject(objectPose,objectAxes); //ground truth object

    //Make initial guess
    Point3 initialGuessCenter(0.1,0.1,0.1);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.2);
    Point2 initialGuessAxes(2.7,0.9);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    mserObject initialGuess(initialGuessPose, initialGuessAxes);

    //Check correctness
    Values correct;
    correct.insert(Symbol('o',0),groundTruthObject);
    Values result = expressionsOptimization(groundTruthObject, initialGuess);

    if (correct.equals(result,0.1)){
        cout << "EXPRESSIONS OPTIMIZATION PASSED" << endl;
    } else {
        cout << "EXPRESSIONS OPTIMIZATION FAILED" << endl;
        correct.print("CORRECT OBJECT");
        result.print("RETURNED OBJECT");
    }
}

void testExpressionsOptimizationWithBadInitialGuess(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject groundTruthObject(objectPose,objectAxes); //ground truth object

    //Make initial guess
    Point3 initialGuessCenter(0.2,-0.5,0.5);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.5);
    Point2 initialGuessAxes(1.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    mserObject initialGuess(initialGuessPose, initialGuessAxes);

    //Check correctness
    Values correct;
    correct.insert(Symbol('o',0),groundTruthObject);
    Values result = expressionsOptimization(groundTruthObject, initialGuess);

    if (correct.equals(result,0.1)){
        cout << "EXPRESSIONS OPTIMIZATION W/ BAD GUESS PASSED" << endl;
    } else {
        cout << "EXPRESSIONS OPTIMIZATION W/ BAD GUESS FAILED" << endl;
        correct.print("CORRECT OBJECT");
        result.print("RETURNED OBJECT");
    }
}

void testMeasurementFunction(){
    //Make object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject object(objectPose,objectAxes);
    //Make camera
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, objectCenter, up, *K);
    //Jacobian matrices
    Eigen::MatrixXd Dcamera(5,11);
    Matrix58 Dobject;
    //Take measurement
    mserMeasurement returnedMeasurement = measurementFunction(camera, object, Dcamera, Dobject);
    //Hand calculated measurement
    mserMeasurement correctMeasurement(Pose2(320,240,0),Point2(150,50));
    //Check correctness
    if (gtsam::traits<mserMeasurement>::Equals(correctMeasurement,returnedMeasurement,0.001)){
        cout << "MEASUREMENT FUNCTION TEST #1 PASSED" << endl;
    } else {
        cout << "MEASUREMENT FUNCTION TEST #1 FAILED" << endl;
        gtsam::traits<mserMeasurement>::Print(correctMeasurement, "CORRECT");
        gtsam::traits<mserMeasurement>::Print(returnedMeasurement, "RETURNED");
    }
}

void testAllMeasurementFunction(){
    testMeasurementFunction();
    testExpressionsOptimization();
    testExpressionsOptimizationWithBadInitialGuess();
}

#endif //MSER_3D_TESTMEASUREMENTFUNCTION_H
