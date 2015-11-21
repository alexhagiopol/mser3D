//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTVISUALIZATION_H
#define MSER_3D_TESTVISUALIZATION_H

#include "Visualizer.h"
#include "geometryFunctions.h"
#include "mserClasses.h"
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


void testAllVisualization(){
    testLocateObject();
    testPointPairOptimize();
    pointPairOptimize();
    testGraphics();
    testProduceMSERMeasurements();
    testBasicMSEROptimization();
}



#endif //MSER_3D_TESTVISUALIZATION_H
