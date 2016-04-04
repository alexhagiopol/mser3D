//
// Created by alex on 3/23/16.
//

#include "measurementFunction.h"
#include "PointsPose.h"
#include "WorldPoints.h"
#include "CameraPoints.h"
#include "TripleManifold.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <CppUnitLite/TestHarness.h>
using namespace gtsam;

TEST(measurementFunction, convertObjectToPointsPose){
    Eigen::MatrixXd H1(12,8);
    boost::function<PointsPose(const MserObject&)> f = boost::bind(&convertObjectToPointsPose, _1, boost::none);
    //Make object
    const Point3 objectCenter(0,0,0);
    const Rot3 objectOrientation(1,0,0,
                                 0,1,0,
                                 0,0,1);
    const Point2 objectAxes(3,1);
    const Pose3 objectPose(objectOrientation, objectCenter);
    const MserObject object(objectPose,objectAxes);
    const PointsPose objectPointsPose = convertObjectToPointsPose(object, H1);
    EXPECT(assert_equal(gtsam::traits<PointsPose>::majAxisTip(objectPointsPose), Point3(3,0,0)));
    EXPECT(assert_equal(gtsam::traits<PointsPose>::minAxisTip(objectPointsPose), Point3(0,1,0)));
    EXPECT(assert_equal(gtsam::traits<PointsPose>::objectPose(objectPointsPose), Pose3()));
    EXPECT(assert_equal(numericalDerivative11(f,object),H1));
}

TEST(measurementFunction, convertPointsPoseToWorldPoints){
    Eigen::MatrixXd H1(9,12);
    boost::function<WorldPoints(const PointsPose&)> f = boost::bind(&convertPointsPoseToWorldPoints, _1, boost::none);
    const PointsPose input = PointsPose(Pose3(), Point3(2,0,0), Point3(0,1,0));
    const WorldPoints actual = convertPointsPoseToWorldPoints(input, H1);
    WorldPoints correct = WorldPoints(Point3(0,0,0), Point3(2,0,0), Point3(0,1,0));
    EXPECT(assert_equal(actual,correct));
    EXPECT(assert_equal(numericalDerivative11(f,input),H1));
}

TEST(measurementFunction, convertWorldPointsToCameraPoints){
    Eigen::MatrixXd H1(6,9);
    Eigen::MatrixXd H2(6,11);
    boost::function<CameraPoints(const SimpleCamera&, const WorldPoints&)> f = boost::bind(&convertWorldPointsToCameraPoints, _1, _2, boost::none, boost::none);
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    const WorldPoints input = WorldPoints(Point3(0,0,0), Point3(2,0,0), Point3(0,1,0));
    Point3 up = Point3(0,1,0);
    Point3 camPosition = Point3(10,0,0);
    const SimpleCamera camera = SimpleCamera::Lookat(camPosition, gtsam::traits<WorldPoints>::centroid(input), up, *K);
    CameraPoints actual = convertWorldPointsToCameraPoints(camera, input, H1, H2);
    EXPECT(assert_equal(numericalDerivative21(f,camera,input),H1,1e-7));
    EXPECT(assert_equal(numericalDerivative22(f,camera,input),H2,1e-7));
}

TEST(measurementFunction, convertCameraPointsToMeasurement){
    Eigen::MatrixXd H1(5,6);
    boost::function<MserMeasurement(const CameraPoints&)> f = boost::bind(&convertCameraPointsToMeasurement, _1, boost::none);
    double theta = 25*M_PI/180;
    double majLength = 20;
    double minLength = 10;
    const CameraPoints input = CameraPoints(Point2(0,0),Point2(majLength*cos(theta),majLength*sin(theta)),Point2(minLength*cos(theta+M_PI/2),minLength*sin(theta+M_PI/2)));
    MserMeasurement actual = convertCameraPointsToMeasurement(input, H1);
    EXPECT(assert_equal(numericalDerivative11(f,input),H1,1e-7));
}

TEST(measurementFunction, measurementFunction){
    Eigen::MatrixXd H1(5,11);
    Eigen::MatrixXd H2(5,8);
    boost::function<MserMeasurement(const SimpleCamera&, const MserObject&)> f = boost::bind(&measurementFunction, _1, _2, boost::none, boost::none);
    const Point3 objectCenter(0,0,0);
    const Rot3 objectOrientation(1,0,0,
                                 0,1,0,
                                 0,0,1);
    const Point2 objectAxes(30,10);
    const Pose3 objectPose(objectOrientation, objectCenter);
    const MserObject object(objectPose,objectAxes);

    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 up = Point3(0,1,0);
    Point3 camPosition = Point3(0,0,10);
    const SimpleCamera camera = SimpleCamera::Lookat(camPosition, objectCenter, up, *K);

    MserMeasurement measurement = measurementFunction(camera,object,H1,H2);
    EXPECT(assert_equal(numericalDerivative21(f,camera,object),H1,1e-5));
    EXPECT(assert_equal(numericalDerivative22(f,camera,object),H2,1e-5));
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}