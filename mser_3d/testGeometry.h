//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTGEOMETRY_H
#define MSER_3D_TESTGEOMETRY_H
#include "measurementFunction.h"
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


void testAllGeometry(){
    //testPointPairOptimize();
    //testLocateObject();
    testEllipse2DOrientation();
}

#endif //MSER_3D_TESTGEOMETRY_H
