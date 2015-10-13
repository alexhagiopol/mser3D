

//#include "Visualizer.h"
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

std::vector<gtsam::SimpleCamera> createCameras(int numCams){
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));
    std::vector<gtsam::SimpleCamera> cameras;
    gtsam::Point3 target(0,0,0);
    gtsam::Point3 up(0,0,1);
    double theta = 0.0;
    double radius = 30;
    for (int i = 0; i < numCams; i++){
        gtsam::Point3 position(radius*cos(theta), radius*sin(theta),0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up, *K);
        cameras.push_back(camera);
        theta += 2*M_PI / numCams;
    }
    return cameras;
}

std::vector<gtsam::Point3> createPoints() {

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.push_back(gtsam::Point3(10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,10.0));
    points.push_back(gtsam::Point3(10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,10.0,-10.0));
    points.push_back(gtsam::Point3(-10.0,-10.0,-10.0));
    points.push_back(gtsam::Point3(10.0,-10.0,-10.0));

    return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses() {

    // Create the set of ground-truth poses
    std::vector<gtsam::Pose3> poses;
    double radius = 30.0;
    int i = 0;
    double theta = 0.0;
    gtsam::Point3 up(0,0,1);
    gtsam::Point3 target(0,0,0);
    for(; i < 8; ++i, theta += 2*M_PI/8) {
        gtsam::Point3 position = gtsam::Point3(radius*cos(theta), radius*sin(theta), 0.0);
        gtsam::SimpleCamera camera = gtsam::SimpleCamera::Lookat(position, target, up);
        poses.push_back(camera.pose());
    }
    return poses;
}

using namespace gtsam;
using namespace std;

class mserMeasurement : public Point2 {
public:
    mserMeasurement(double x, double y, double theta, double majorAxis, double minorAxis): theta_(theta), majorAxis_(majorAxis), minorAxis_(minorAxis){}
    double theta() {return theta_;}
    double majorAxis() {return majorAxis_;}
    double minorAxis() {return minorAxis_;}
private:
    double theta_;
    double majorAxis_;
    double minorAxis_;
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

int main() {
	//Extract VO data from video - save this for later when we make a camera fly through a real scene
    /*
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
        const Pose3 pose = camera.pose();
        pose.print("Camera pose:\n");
    }
     */
    /* //Standard factor
    int numPoints = 10;
    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr noise = noiseModel::Diagonal::Sigmas(Vector3(0.3,0.3,0.3));
    for (int i = 0; i < numPoints; i++){
        Point3 point(0.0,0.0,0.0 + i);
        graph.add(PriorFactor<Point3>(1, point, noise));
    }
     */

    /* //Expressions graph work - need more tim to understand
    gtsam::Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);
    typedef gtsam::SimpleCamera Camera;
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v
    gtsam::ExpressionFactorGraph graph;
    gtsam::Point3 target(0.0,0.0,0.0);

    gtsam::Vector6 sigmas;
    sigmas << gtsam::Vector3(0.1,0.1,0.1), gtsam::Vector3(0.1,0.1,0.1);
    gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    gtsam::Pose3_ x0('x',0);
    graph.addExpressionFactor(x0, cameras[0].pose(), noise);
    gtsam::Cal3_S2_ cK(K);


    for (size_t i = 0; i < cameras.size(); i++){
        gtsam::Pose3_ x('x',i);
        Camera camera = cameras[i];
        gtsam::Point2 measurement = camera.project(target);  //need MSER representation here
        gtsam::Point3_ p('l', 0); //not sure about this...i only have 1 target point
        gtsam::Point2_ prediction = gtsam::uncalibrate(cK, project(transform_to(x, p)));
        graph.addExpressionFactor(prediction, measurement, measurementNoise);
    }

    // Add prior on first point to constrain scale, again with ExpressionFactor
    gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    graph.addExpressionFactor(gtsam::Point3_('l', 0), target, pointNoise);
    */

    /*
    Values initial;
    initial.insert(1,Point3(0.1,0.0,8.0));
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Result \n");
    */

    int numMeasurements = 8;
    std::vector<SimpleCamera> cameras = createCameras(numMeasurements); //from my camera pose generating function - soon replace with camera poses converted from OpenGL flythrough
    //indeterminate linear system for one target point .... ?
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)); //made up calibration for now; replace with Jing's calibration later
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
    Point3 target1(0,0,0); //assume just one point in center of world...this will be the centroid of the object which will be observed with MSER measurements
    Point3 target2(0,0,0.5);
    std::vector<Point3> targets;
    targets.push_back(target1);
    targets.push_back(target2);
    NonlinearFactorGraph graph;
    Vector6 sigmas;
    //sigmas << Vector3(0.3,0.3,0.3), Vector3(0.1,0.1,0.1); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw

    //noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas(sigmas);
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), cameras[0].pose(), poseNoise)); // add directly to graph

    for (size_t i = 0; i < cameras.size(); ++i) {
        for (size_t j = 0; j < targets.size(); i++){
            Point2 measurement = cameras[i].project(targets[j]);
            graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
        }
    }

    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.push_back(PriorFactor<Point3>(Symbol('l', 0), targets[0], pointNoise));

    graph.print("Factor Graph:\n");
    Values initialEstimate;
    for (size_t i = 0; i < cameras.size(); ++i){
        initialEstimate.insert(Symbol('x', i), cameras[i].pose().compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    }
    for (size_t j = 0; j < targets.size(); ++j){
        initialEstimate.insert(Symbol('l', 0), targets[j].compose(Point3(-0.25, 0.20, 0.15)));
    }

    // Optimize the graph and print results
    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");
    std::cout << "initial error = " << graph.error(initialEstimate) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;

    //visualize(50);
    return 0;
}
