//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTVISUALIZATION_H
#define MSER_3D_TESTVISUALIZATION_H

#include "geometryFunctions.h"
#include "measurementFunction.h"
#include "Visualizer.h"
#include <gtsam/base/Manifold.h> //required for MSER object

using namespace gtsam;
using namespace std;
using namespace noiseModel;


void testMserObjectDrawing(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject groundTruthObject(objectPose,objectAxes);
    //Make initial guess
    Point3 initialGuessCenter(0.2,1.5,-0.5);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.5);
    Point2 initialGuessAxes(1.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    MserObject initialGuess(initialGuessPose, initialGuessAxes);
    //Get measurements as well
    std::vector<SimpleCamera> cameras = alexCreateCameras(15,groundTruthObject.first.translation(),20); //make a bunch of cameras to pass to measurement function
    std::vector<MserMeasurement> measurements = createIdealMeasurements(cameras, groundTruthObject); //synthetic measurements directly from measurement function
    //std::vector<Pose3> poses;
    std::vector<MserObject> measurementsAsObjects; //needed to draw both measurements and resulting optimize object on same screen
    for (size_t i = 0; i < cameras.size(); i++){
        //poses.push_back(cameras[i].pose());
        MserObject temporary(cameras[i].pose(),measurements[i].second/30);
        measurementsAsObjects.push_back(temporary);
    }

    //Optimize
    Values result = expressionsOptimizationSynthetic(groundTruthObject, initialGuess);
    MserObject returnedObject = result.at<MserObject>(Symbol('o',0)); //retrieve object
    std::vector<MserObject> objects;
    objects.push_back(groundTruthObject);
    objects.push_back(returnedObject);
    objects.push_back(initialGuess);
    objects.insert(objects.end(),measurementsAsObjects.begin(),measurementsAsObjects.end());
    drawMserObjects(objects);
}

void testAllVisualization(){
    testMserObjectDrawing();
}

#endif //MSER_3D_TESTVISUALIZATION_H
