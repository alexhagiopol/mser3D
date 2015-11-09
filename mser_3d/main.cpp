#include "Visualizer.h"
#include "alexFunctions.h"
#include <gtsam/base/Manifold.h> //required for MSER object
#include <gtsam/geometry/Pose3.h> //required for MSER object
#include <gtsam/geometry/Pose2.h> //required for MSER measurement
#include <gtsam/geometry/Point2.h> //required for MSER object
#include <gtsam/slam/expressions.h> //required for optimization with Expressions syntax

using namespace gtsam;
using namespace std;
using namespace noiseModel;

namespace gtsam { //required to keep GCC from complaining
    typedef ProductManifold<Pose3, Point2> mserObject;

    template<>
    struct traits<mserObject> : internal::ManifoldTraits<mserObject> {
        static void Print(const mserObject &o, const string &s = "") {
            cout << s << " (";
            o.first.print();
            cout << ",";
            o.second.print();
            cout << ")" << endl;
            //cout << s << "(" << o.first << "," << o.second << ")" << endl;
        }

        static bool Equals(const mserObject &o1, const mserObject &o2, double tol = 1e-8) {
            return ((o1.first.equals(o2.first)) && (o1.second.equals(o2.second)));
        }
    };

    typedef ProductManifold<Pose2, Point2> mserMeasurement;

    template<>
    struct traits<mserMeasurement> : internal::ManifoldTraits<mserMeasurement> {
        static void Print(const mserMeasurement &m, const string &s = "") {
            cout << s << " (";
            m.first.print();
            cout << ",";
            m.second.print();
            cout << ")" << endl;
            //cout << s << "(" << m.first << "," << m.second << ")" << endl;
        }

        static bool Equals(const mserMeasurement &m1, const mserMeasurement &m2, double tol = 1e-8) {
            return ((m1.first.equals(m2.first)) && (m1.second.equals(m2.second)));
        }
    };

    typedef ProductManifold<Point2,Point2> MyPoint2Pair;

    // Define any direct product group to be a model of the multiplicative Group concept
    template<> struct traits<MyPoint2Pair> : internal::ManifoldTraits<MyPoint2Pair> {
        static void Print(const MyPoint2Pair& m, const string& s = "") {
            cout << s << "(" << m.first << "," << m.second << ")" << endl;
        }
        static bool Equals(const MyPoint2Pair& m1, const MyPoint2Pair& m2, double tol = 1e-8) {
            return m1 == m2;
        }
    };
}//namespace gtsam

/*
Values SFMExpressionsProductManifolds(){
    typedef Expression<mserObject> mserObject_;
    typedef Expression<mserMeasurement> mserMeasurement_;
    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();
    vector<mserObject>
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
    //measurement noise
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
    vector<Point3> points = createPoints();
    ExpressionFactorGraph graph;
    //pose noise
    Vector6 sigmas; sigmas << Vector3(0.3,0.3,0.3), Vector3(0.1,0.1,0.1);
    Diagonal::shared_ptr poseNoise = Diagonal::Sigmas(sigmas);
    mserObject_ x0('x',0);
    graph.addExpressionFactor(x0, poses[0], poseNoise);

}
 */

//Example optimization using a new object that consists of a pair of Point2 objects.
//Essentially computes the average of a set of these objects to serve as unit test.
//Uses standard GTSAM syntax i.e. no Expressions yet.
Values pointPairOptimize(){
    MyPoint2Pair p1(Point2(1,2),Point2(3,4));
    MyPoint2Pair p2(Point2(10,20),Point2(30,40));
    NonlinearFactorGraph graph;
    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(4, 0.1);
    graph.add(PriorFactor<MyPoint2Pair>(1,p1,pointNoise));
    graph.add(PriorFactor<MyPoint2Pair>(1,p2,pointNoise));
    Values initial;
    initial.insert(1, p1);
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print();
    return result;
}

//Example optimization using 3D points. Given a set of data points
Values locateObject(Point3 target, Point3 guess, int numCams, double radius){
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    //produceMSERMeasurements(cameras); //virtual stuff in opengl
    NonlinearFactorGraph graph;
    for (int i = 0; i < numCams; i++){
        Point2 measurement = cameras[i].project(target);
        noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
        Point3 backProjectedPoint3 = cameras[i].backproject(measurement, radius); //assume depth is known from other methods e.g. VO
        graph.add(PriorFactor<Point3>(1, backProjectedPoint3, pointNoise));
    }
    //estimate object centroid location
    Values initial;
    initial.insert(1, guess);
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    //result.print();
    return result;
}

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

//Ensures that graphics pipeline does not throw errors. Outputs 5 images with random cubes into output folder.
void testGraphics(){
    int success = produceRandomCubeImages(5);
    if (success == 0){
        cout << "Graphics WORKING" << endl;
    }else{
        cout << "Graphics NOT WORKING"<< endl;
    }
}

int main() {
    testLocateObject();
    testPointPairOptimize();
    pointPairOptimize();
    testGraphics();
    return 0;
}





