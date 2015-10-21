#include "Visualizer.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;

/*
class mserMeasurement : public Point2 {
public:
    mserMeasurement(double x, double y, double theta, double majorAxis, double minorAxis): theta_(theta), majorAxis_(majorAxis), minorAxis_(minorAxis){}
    double theta() const {return theta_;}
    double majorAxis() const {return majorAxis_;}
    double minorAxis() const {return minorAxis_;}
    bool equals(const mserMeasurement& q, double tol) const {
        return (fabs(this->x() - q.x()) < tol && fabs(this->y() - q.y()) < tol  && fabs(theta_ - q.theta()) < tol && fabs(majorAxis_ - q.majorAxis()) < tol && fabs(minorAxis_ - q.minorAxis()) < tol);
    }
private:
    double theta_;
    double majorAxis_;
    double minorAxis_;
};

namespace gtsam{
template<>
struct traits<mserMeasurement> : public internal::VectorSpace<Point2> {};
};


class objectPose3 : public Pose3{
public:
    objectPose3(const Pose3& pose, double theta, double majorAxis, double minorAxis): theta_(theta), majorAxis_(majorAxis), minorAxis_(minorAxis){}
    double theta() {return theta_;}
    double majorAxis() {return majorAxis_;}
    double minorAxis() {return minorAxis_;}
private:
    double theta_;
    double majorAxis_;
    double minorAxis_;
};

//Don't like this because it uses Unit3 not Pose3...
class objectPlane : public OrientedPlane3{
public:
    objectPlane(const Unit3& s, double d,  double theta, double majorAxis, double minorAxis): theta_(theta), majorAxis_(majorAxis), minorAxis_(minorAxis){}
    double theta() {return theta_;}
    double majorAxis() {return majorAxis_;}
    double minorAxis() {return minorAxis_;}
private:
    double theta_;
    double majorAxis_;
    double minorAxis_;
};
*/

std::vector<Pose3> alexCreatePoses(double radius, Point3 target, int numPoses){
    std::vector<gtsam::Pose3> poses;
    double theta = 0.0;
    Point3 up = Point3(0,0,1);
    for(int i = 0; i < numPoses; i++){
        Point3 position = Point3(target.x() + radius*cos(theta), target.y() + radius*sin(theta), target.z() + 0.0);
        SimpleCamera tempCam = SimpleCamera::Lookat(position, target, up);
        theta += 2*M_PI/numPoses;
        poses.push_back(tempCam.pose());
    }
    return poses;
}

std::vector<SimpleCamera> alexCreateCameras(double radius, Point3 target, int numCams){
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)); //made up calibration for now; replace with Jing's calibration later
    std::vector<SimpleCamera> cameras;
    double theta = 0.0;
    Point3 up = Point3(0,0,1);
    for(int i = 0; i < numCams; i++){
        Point3 position = Point3(target.x() + radius*cos(theta), target.y() + radius*sin(theta), target.z() + 0.0);
        SimpleCamera tempCam = SimpleCamera::Lookat(position, target, up, *K);
        cameras.push_back(tempCam);
        theta += 2*M_PI/numCams;
    }
    return cameras;
}

void printJingData(){
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
                    const Pose3 pose = camera.pose();
                    pose.print("Camera pose:\n");
                }
}

Values locateObject(Point3 target, Point3 guess, int numCams, double radius){
    std::vector<SimpleCamera> cameras = alexCreateCameras(radius, target, numCams);
    produceMSERMeasurements(cameras);
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
/* //Still not compiling hence the comments
Values averageMSERExperiment(mserMeasurement guess, int numMeasurements){
    NonlinearFactorGraph graph;
    for (int i = 0; i < numMeasurements; i++){
        mserMeasurement tempMsmt = mserMeasurement(0.0,0.0 + i,0.0,0 + i, 5 + i);
        noiseModel::Isotropic::shared_ptr mserNoise = noiseModel::Isotropic::Sigma(5, 0.1);
        graph.add(PriorFactor<mserMeasurement>(1, tempMsmt, mserNoise));
    }
    Values initial;
    initial.insert(1, guess);
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    return result;
}
 */

void testPipeline(){
    //test for object localization via back projection
    Point3 target = Point3(1.5,1.5,1.5);
    Point3 guess = Point3(1.1,1.1,1.1);
    int numCameras = 8;
    double cameraMotionRadius = 10.0;
    Values correct1;
    correct1.insert(1,target);
    Values result1 = locateObject(target, guess, numCameras, cameraMotionRadius);
    if (result1.equals(correct1,0.0001)){
        cout << "\n Localization PASSED. \n" << endl;
    }
    else{
        cout << "\n Localization FAILED. \n" << endl;
    }

    /*
    //test MSER averaging: still not compiling hence the comments
    mserMeasurement guessMSER = mserMeasurement(0.1,3.5,0.1,3.5,7.5);
    int numMeasurements = 10;
    Values correct2;
    correct2.insert(1, mserMeasurement(0.0,4.5,0.0,4.5,9.5));
    Values result2 = averageMSERExperiment(guessMSER, numMeasurements);
    result2.print();
    if (result2.equals(correct2,0.0001)){
        cout << "\n MSER Average PASSED. \n" << endl;
    }
    else{
        cout << "\n MSER Average FAILED. \n" << endl;
    }
     */
}

int main() {
    testPipeline();
    return 0;
}




