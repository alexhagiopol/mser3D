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

void SFMexample(){
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0)); //made up calibration for now; replace with Jing's calibration later
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();

    NonlinearFactorGraph graph;
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << Vector3::Constant(0.3), Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise)); // add directly to graph

    //Simulates measurements
    for (size_t i = 0; i < poses.size(); ++i) {
        SimpleCamera temp_cam(poses[i], *K);
        for (size_t j = 0; j < points.size(); ++j){
            Point2 measurement = temp_cam.project(points[j]);
            graph.push_back(GenericProjectionFactor<Pose3, Point3, Cal3_S2>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K));
        }
    }

    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.push_back(PriorFactor<Point3>(Symbol('l', 0), points[0], pointNoise));

    graph.print("Factor Graph:\n");
    Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i){
        initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    }
    for (size_t j = 0; j < points.size(); ++j){
        initialEstimate.insert(Symbol('l', j), points[j].compose(Point3(-0.25, 0.20, 0.15)));
    }
    initialEstimate.print("Initial Estimates:\n");
    // Optimize the graph and print results
    Values result = DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");
    std::cout << "initial error = " << graph.error(initialEstimate) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;
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

int main() {
    visualize(50);
    return 0;
}




