//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTVISUALIZATION_H
#define MSER_3D_TESTVISUALIZATION_H

#include "Visualizer.h"
#include "geometryFunctions.h"
#include "mserClasses.h"
#include "measurementFunction.h"
#include <gtsam/base/Manifold.h> //required for MSER object

using namespace gtsam;
using namespace std;
using namespace noiseModel;


//Unit test for graphics; ensures that graphics pipeline does not throw exceptions. Outputs 5 images with random cubes into output folder.
void testGraphics(){
    int success = produceRandomCubeImages(5);
    if (success == 0){
        cout << "Graphics WORKING" << endl;
    }else{
        cout << "Graphics NOT WORKING"<< endl;
    }
}

//Test production of MSER measurements from synthetic world model. Still need to verify output...
void testProduceMSERMeasurements(){
    int numCams = 20;
    double radius = 15.0;
    Point3 target = Point3(0.0,0.0,0.0);
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    std::vector<mserMeasurement> measurements;
    int success = produceMSERMeasurements(cameras, target, measurements);
    drawEllipses(cameras, measurements);
    if (success == 0){
        for (size_t i  = 0; i < measurements.size(); i++ ){
            //traits<mserMeasurement>::Print(measurements[i]);
        }
        cout << "MSER measurements PRODUCED" << endl;
    } else {
        cout << "MSER measurements FAILED" << endl;
    }
}
/*
void testBasicMSEROptimization(){ //work in progress
    int numCams = 20;
    double radius = 10.0;
    Point3 target = Point3(0.0,0.0,0.0);
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    std::vector<mserMeasurement> measurements;
    mserObject object;
    int success = produceMSERMeasurements(cameras, target, measurements);
    if ((success != 0) || (measurements.size() != cameras.size())){
        cout << "MSER optimization FAILED because measurement simulation FAILED" << endl;
    } else {
        inferObject(cameras, measurements, object);
        traits<mserObject>::Print(object); //not ready yet
    }
}
*/

void testMserObjectDrawing(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject groundTruthObject(objectPose,objectAxes);
    //Make initial guess
    Point3 initialGuessCenter(0.2,1.5,-0.5);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.5);
    Point2 initialGuessAxes(1.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    mserObject initialGuess(initialGuessPose, initialGuessAxes);
    //Optimize
    Values result = expressionsOptimization(groundTruthObject, initialGuess);
    mserObject returnedObject = result.at<mserObject>(Symbol('o',0)); //retrieve object
    std::vector<mserObject> objects;
    objects.push_back(groundTruthObject);
    objects.push_back(returnedObject);
    objects.push_back(initialGuess);
    drawMserObjects(objects);
}

void testAllVisualization(){
    //testGraphics();
    //testProduceMSERMeasurements();
    testMserObjectDrawing();
}



#endif //MSER_3D_TESTVISUALIZATION_H
