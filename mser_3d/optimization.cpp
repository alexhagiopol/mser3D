//
// Created by alex on 2/16/16.
//

#include "MserObject.h"
#include "MserMeasurement.h"
#include "MserTrack.h"
#include "geometryFunctions.h"
#include "measurementFunction.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/triangulation.h>
#include "boost/optional.hpp"

#include "optimization.h"
using namespace gtsam;
using namespace noiseModel;

Values expressionsOptimization(MserObject& initialGuess, std::vector<MserMeasurement>& measurements, std::vector<SimpleCamera>& cameras, int iterations){
    Vector5 measurementSigmasVector;
    measurementSigmasVector << 4, 4, 0.1, 4, 4;
    Diagonal::shared_ptr measurementNoise = Diagonal::Sigmas(measurementSigmasVector); // one pixel in every dimension

    //Ground truth object is passed to this function. Create vectors with measurements, cameras, and camera poses.
    //std::vector<SimpleCamera> cameras = alexCreateCameras(20,object.first.translation(),20); //make a bunch of cameras to pass to measurement function
    //std::vector<MserMeasurement> measurements = createIdealMeasurements(cameras, object); //synthetic measurements directly from measurement function
    std::vector<Pose3> poses;
    for (size_t i = 0; i < cameras.size(); i++){
        poses.push_back(cameras[i].pose());
    }

    // Create a factor graph
    ExpressionFactorGraph graph;

    for (size_t i = 0; i < poses.size(); ++i) {
        const SimpleCamera_ camera_(cameras[i]); //expression for the camera created here
        MserMeasurement measurement = measurements[i];
        // Below an expression for the prediction of the measurement:
        MserObject_ object_('o',0);
        MserMeasurement_ prediction_ = measurementFunctionExpr(camera_,object_);
        graph.addExpressionFactor(prediction_,measurement,measurementNoise);
    }
    // Add prior on object to constrain scale, again with ExpressionFactor[
    Vector8 guessSigmasVector;
    guessSigmasVector << 0.1,0.1,0.1,1,1,1,5,5;
    Diagonal::shared_ptr objectNoise = Diagonal::Sigmas(guessSigmasVector);
    graph.addExpressionFactor(MserObject_('o', 0), initialGuess, objectNoise); //use initial guess as prior

    // Create perturbed initial
    Values initial;
    initial.insert(Symbol('o', 0), initialGuess);
    //cout << "initial error = " << graph.error(initial) << endl;
    LevenbergMarquardtParams params = LevenbergMarquardtParams();
    std::string verbosity = "SUMMARY";
    params.setVerbosityLM(verbosity);
    params.setlambdaUpperBound(1e32);
    params.setMaxIterations(iterations);
    LevenbergMarquardtOptimizer optimizer = LevenbergMarquardtOptimizer(graph,initial,params);
    Values result = optimizer.optimize();//LevenbergMarquardtOptimizer(graph, initial).optimize();
    //cout << "final error = " << graph.error(result) << endl;
    return result;
}

//Helper function ofr showing measurement stats
float standardDeviation(std::vector<float> data)
{
    int n = data.size();
    float mean=0.0, sum_deviation=0.0;
    int i;
    for(i=0; i<n;++i)
    {
        mean+=data[i];
    }
    mean=mean/n;
    for(i=0; i<n;++i)
        sum_deviation+=(data[i]-mean)*(data[i]-mean);
    return sqrt(sum_deviation/n);
}

std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(std::vector<MserTrack>& tracks, std::vector<Pose3>& VOposes){
    std::vector<MserObject> objects;
    std::vector<Vector3> colors;
    Cal3_S2::shared_ptr K(new Cal3_S2(857.483, 876.718, 0.1, 1280/2, 720/2)); //gopro camera calibration from http://www.theeminentcodfish.com/gopro-calibration/
    float sumMeasurements; //use for measurement stats
    std::vector<float> measurementsSizes; //use for measurement stats
    for (int t = 0; t < tracks.size(); t++){
        std::vector<MserMeasurement> measurements = tracks[t].measurements;
        std::cerr << "OPTIMIZER: Track #" << t << " has " << measurements.size() << " measurements." << std::endl;
        std::vector<SimpleCamera> cameras;
        std::vector<Point2> centroidMeasurements;
        std::vector<Point2> majorAxisMeasurements;
        std::vector<Point2> minorAxisMeasurements;
        sumMeasurements += measurements.size();
        measurementsSizes.push_back(measurements.size());
        for (int m = 0; m < measurements.size(); m++){
            MserMeasurement measurement = tracks[t].measurements[m];
            //make cameras with poses from Jing's Visual Odometry
            Pose3 pose = VOposes[tracks[t].frameNumbers[m]];
            SimpleCamera camera(pose,*K);
            cameras.push_back(camera);
            //extract contents of measurement
            double theta = measurement.first.theta();
            double majAxis  = measurement.second.x();
            double minAxis = measurement.second.y();
            double ctrX = measurement.first.translation().x();
            double ctrY = measurement.first.translation().y();

            //make vector of 2D centroid measurements
            centroidMeasurements.push_back(measurement.first.translation());
            //measurement.first.print("MSER MEASUREMENT POSE \n");
            //measurement.second.print("MSER MEASUREMENT AXES \n");

            //vector of 2D major axis measurements
            double majX = ctrX + majAxis*cos(theta);
            double majY = ctrY + majAxis*sin(theta);
            Point2 majorAxisMeasurement(majX,majY);
            majorAxisMeasurements.push_back(majorAxisMeasurement);
            //majorAxisMeasurement.print("MAJOR AXIS MEASUREMENT \n");

            //vector of 2D minor axis measurements
            double minX = ctrX + minAxis*cos(theta + M_PI/2);
            double minY = ctrY + minAxis*sin(theta + M_PI/2);
            Point2 minorAxisMeasurement(minX,minY);
            minorAxisMeasurements.push_back(minorAxisMeasurement);
            //minorAxisMeasurement.print("MINOR AXIS MEASUREMENT \n");
            //cout << "THETA \n" << theta << endl;
        }

        //Initial guess via triangulation.h methods
        Point3 initialGuessCentroid = gtsam::triangulatePoint3(cameras,centroidMeasurements,1e-9,true);
        Point3 initialGuessMajAxisPoint = gtsam::triangulatePoint3(cameras,majorAxisMeasurements,1e-9,true);
        Point3 initialGuessMinAxisPoint = gtsam::triangulatePoint3(cameras,minorAxisMeasurements,1e-9,true);
        //initialGuessCentroid.print("CENTROID GUESS \n");
        //initialGuessMajAxisPoint.print("MAJ AXIS POINT GUESS \n");
        //initialGuessMinAxisPoint.print("MIN AXIS POINT GUESS \n");
        Rot3 initialGuessOrientation = cameras[0].pose().rotation();
        Pose3 initialGuessPose = Pose3(initialGuessOrientation, initialGuessCentroid);
        double majAxisLengthEstimate = initialGuessCentroid.distance(initialGuessMajAxisPoint);
        double minAxisLengthEstimate = initialGuessCentroid.distance(initialGuessMinAxisPoint);
        Point2 initialGuessAxes = Point2(majAxisLengthEstimate,minAxisLengthEstimate);
        //initialGuessAxes.print("AXES LENGTH GUESS");
        MserObject initialGuess(initialGuessPose, initialGuessAxes);
        Values result = expressionsOptimization(initialGuess, measurements, cameras, 30);
        MserObject returnedObject = result.at<MserObject>(Symbol('o',0));
        objects.push_back(returnedObject);
        //gtsam::traits<MserObject>::Print(returnedObject);
        //gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
        gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
        colors.push_back(trackColor);
        cerr << "OPTIMIZER: Finished optimizing track #" << t << " of " << tracks.size() - 1 << endl;
    }
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair(objects,colors);
    cerr << "OPTIMIZER: Average # of measurements / track = " << sumMeasurements / tracks.size() << endl;
    cerr << "OPTIMIZER: STDEV of # of measurements / track = " << standardDeviation(measurementsSizes) << endl;
    return pair;
}