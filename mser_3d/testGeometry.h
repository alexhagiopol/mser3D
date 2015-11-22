//
// Created by Alex Hagiopol on 11/20/15.
//

#ifndef MSER_3D_TESTGEOMETRY_H
#define MSER_3D_TESTGEOMETRY_H
#include "geometryFunctions.h"
#include "mserClasses.h"

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

void testATAN2(){
    Point2 p1(5,5);
    Point2 p2(10,10);

}

void testAllGeometry(){
    testPointPairOptimize();
    testLocateObject();
}

#endif //MSER_3D_TESTGEOMETRY_H
