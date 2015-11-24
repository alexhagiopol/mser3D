//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTGEOMETRY_H
#define MSER_3D_TESTGEOMETRY_H
#include "geometryFunctions.h"
#include "mserClasses.h"
#include <gtsam/nonlinear/Values.h>

using namespace gtsam;
using namespace std;

//Unit test for pointPairOptimize()
void testPointPairOptimize(){
    MyPoint2Pair pair1;
    Vector4 d;
    d << 1,2,3,4;
    MyPoint2Pair expected(Point2(1,2),Point2(3,4));
    MyPoint2Pair pair2 = pair1.retract(d);
    if (gtsam::traits<MyPoint2Pair>::Equals(expected, pair2)){
        std::cout << "Point2 Pair Example PASSED." << std::endl;
    }
    else{
        std::cout << "Point2 Pair Example FAILED." << std::endl;
    }
}

//Unit test for 3D object location
void testLocateObject(){
    //test for object localization via back projection
    Point3 target = Point3(1.5,1.5,1.5);
    Point3 guess = Point3(1.1,1.1,1.1);
    int numCameras = 8;
    double cameraMotionRadius = 10.0;
    Values correct1;
    correct1.insert(1,target);
    Values result1 = locateObject(target, guess, numCameras, cameraMotionRadius);
    if (result1.equals(correct1,0.0001)){
        cout << "Localization PASSED." << endl;
    }
    else{
        cout << "Localization FAILED." << endl;
    }
}

void testEllipse2DOrientation(){
    Point2 center(5,5);
    Point2 majorAxisTip(10,10);
    Matrix12 H1, H2, correctCenterJacobian, correctAxisPointJacobian;
    double orientation = ellipse2DOrientation(center, majorAxisTip, H1, H2);
    //cout << orientation << H1 << H2 << endl;
    correctCenterJacobian << 0.1, -0.1; //hand calculated value
    correctAxisPointJacobian << -0.1, 0.1; //hand calculated value
    double correctOrientation = 0.7854; //(radians) hand calculated value
    Values correct;
    Values tested;
    correct.insert(1, correctCenterJacobian);
    correct.insert(2, correctAxisPointJacobian);
    correct.insert(3, correctOrientation);
    tested.insert(1, H1);
    tested.insert(2, H2);
    tested.insert(3, orientation);
    if (correct.equals(tested, 0.00001)){
        cout << "ellipse2DOrientation PASSED" << endl;
    } else {
        cout << "ellipse2DOrientation FAILED" << endl;
        correct.print();
        tested.print();
    }
}

void testMserMeasurementFunction(){

    //object parameters
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    mserObject object(objectPose,objectAxes);

    //compute 3D points from object Pose and object axes
    Point3 majorAxisTip = objectOrientation.rotate(Point3(objectAxes.x(),0,0)) + objectCenter;
    Point3 minorAxisTip = objectOrientation.rotate(Point3(0,objectAxes.y(),0)) + objectCenter;


    //We now have a representation of the mser Object as a set of three 3D points.
    //next step is to project each point into 2D by making a camera
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, objectCenter, up, *K);
    Point2 projectedObjectCenter = camera.project(objectCenter);
    Point2 projectedMajorAxisTip = camera.project(majorAxisTip);
    Point2 projectedMinorAxisTip = camera.project(minorAxisTip);
    //projectedObjectCenter.print();
    //projectedMajorAxisTip.print();
    //projectedMinorAxisTip.print();
}

void testConvertObjectToPoint3s(){
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
    Matrix98 H;
    std::vector<Point3> pointVector = convertObjectToPoint3s(object,H);
    Values correctPoints, returnedPoints;
    correctPoints.insert(1,objectCenter);
    correctPoints.insert(2,majorAxisTip);
    correctPoints.insert(3,minorAxisTip);
    returnedPoints.insert(1,pointVector[0]);
    returnedPoints.insert(2,pointVector[1]);
    returnedPoints.insert(3,pointVector[2]);
    if (correctPoints.equals(returnedPoints, 0.00001)){
        cout << "convertObjectToPoint3s PASSED" << endl;
    } else {
        cout << "convertObjectToPoint3s FAILED" << endl;
        correctPoints.print();
        returnedPoints.print();
    }
}

void testConvertPoint3sToMeasurement(){
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
    Matrix98 H;
    std::vector<Point3> pointVector = convertObjectToPoint3s(object,H);
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, objectCenter, up, *K);
    mserMeasurement returnedMeasurement = convertPoint3sToMeasurement(camera, pointVector);
    gtsam::traits<mserMeasurement>::Print(returnedMeasurement);
}



void testAllGeometry(){
    testPointPairOptimize();
    testLocateObject();
    testEllipse2DOrientation();
    testMserMeasurementFunction();
    testConvertObjectToPoint3s();
    testConvertPoint3sToMeasurement();
}

#endif //MSER_3D_TESTGEOMETRY_H
