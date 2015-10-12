#include "Visualizer.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/slam/GeneralSFMFactor.h>

using namespace gtsam;



int main() {
	//Extract VO data from video - save this for later when we make a camera fly through the scene
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

    int numMeasurements = 10;
    ExpressionFactorGraph graph;
    noiseModel::Diagonal::Sigmas(Vector3(0.1,0.1,0.1));
    SimpleCamera knownCameras[5];
    Point2 centroids[5];



    Values initial;
    initial.insert(1,Point3(0.1,0.0,8.0));
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Result \n");
    //visualize(50);
    return 0;
}