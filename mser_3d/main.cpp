#include "Visualizer.h"
#include "geometryFunctions.h"
#include "mserClasses.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/Manifold.h> //required for MSER object

using namespace gtsam;
using namespace std;
using namespace noiseModel;

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
void testMSERMeasurements(){
    int numCams = 20;
    double radius = 10.0;
    Point3 target = Point3(0.0,0.0,0.0);
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    std::vector<mserMeasurement> measurements;
    int success = produceMSERMeasurements(cameras, target, measurements);
    if (success == 0){
        for (size_t i  = 0; i < measurements.size(); i++ ){
            //traits<mserMeasurement>::Print(measurements[i]);
        }
        cout << "MSER measurements PRODUCED" << endl;
    } else {
        cout << "MSER measurements FAILED" << endl;
    }
}

void testNaiveMSEROptimization(){
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

int main() {
    testLocateObject();
    testPointPairOptimize();
    pointPairOptimize();
    testGraphics();
    testMSERMeasurements();
    testNaiveMSEROptimization();
    drawEllipse();
    return 0;
}





