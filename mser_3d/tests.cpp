//
// Created by alex on 2/24/16.
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

/*
TEST(measurementFunction, convertWorldPointsToCameraPoints){
    Eigen::MatrixXd H1(6,6);C
    Eigen::MatrixXd H2(6,5);
    Eigen::MatrixXd H3(6,9);
    boost::function<CameraPoints(const SimpleCamera&, const WorldPoints&)> f = boost::bind(&convertWorldPointsToCameraPoints, _1, _2, boost::none, boost::none);
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    const WorldPoints input = WorldPoints(Point3(0,0,0), Point3(2,0,0), Point3(0,1,0));
    Point3 up = Point3(0,1,0);
    Point3 camPosition = Point3(10,0,0);
    const SimpleCamera camera = SimpleCamera::Lookat(camPosition, gtsam::traits<WorldPoints>::centroid(input), up, *K);

    CameraPoints actual = convertWorldPointsToCameraPoints(camera, input, H1, H2, H3);
    EXPECT(assert_equal(numericalDerivative31(f,camera,input),H1));
}
*/

TEST(measurementFunction, measurementFunction){
    Eigen::MatrixXd H1(5,1);
    Eigen::MatrixXd H2(5,8);
    boost::function<MserMeasurement(const SimpleCamera&, const MserObject&)> f = boost::bind(&measurementFunction, _1, _2, boost::none, boost::none);
    const Point3 objectCenter(0,0,0);
    const Rot3 objectOrientation(1,0,0,
                                 0,1,0,
                                 0,0,1);
    const Point2 objectAxes(3,1);
    const Pose3 objectPose(objectOrientation, objectCenter);
    const MserObject object(objectPose,objectAxes);

    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 up = Point3(0,1,0);
    Point3 camPosition = Point3(10,0,0);
    const SimpleCamera camera = SimpleCamera::Lookat(camPosition, objectCenter, up, *K);

    MserMeasurement measurement = measurementFunction(camera,object,H1,H2);
    EXPECT(assert_equal(numericalDerivative21(f,camera,object),H1));
    EXPECT(assert_equal(numericalDerivative22(f,camera,object),H2));
}

TEST(PointsPose, manifold){
    BOOST_CONCEPT_ASSERT((internal::HasManifoldPrereqs<PointsPose>));
    BOOST_CONCEPT_ASSERT((IsManifold<PointsPose>));
    Pose3 objectPose;
    Point3 majAxisTip(1,2,3);
    Point3 minAxisTip(4,5,6);
    const PointsPose a(objectPose,majAxisTip,minAxisTip);
    const PointsPose b(objectPose,majAxisTip + Point3(1,2,3),minAxisTip);
    Vector zero = Vector::Zero(12);
    EXPECT(assert_equal(zero,a.localCoordinates(a)));
    EXPECT(assert_equal(a,a.retract(zero)));
    Vector12 v = a.localCoordinates(b);
    PointsPose c  = a.retract(v);
    EXPECT(assert_equal(b,c));
    EXPECT(check_manifold_invariants(a,b));
}

TEST(WorldPoints, manifold){
    BOOST_CONCEPT_ASSERT((internal::HasManifoldPrereqs<WorldPoints>));
    BOOST_CONCEPT_ASSERT((IsManifold<WorldPoints>));
    Point3 centroid(0,0,0);
    Point3 majAxisTip(1,2,3);
    Point3 minAxisTip(4,5,6);
    const WorldPoints a(centroid, majAxisTip, minAxisTip);
    const WorldPoints b(centroid, majAxisTip + Point3(1,2,3), minAxisTip);
    Vector zero = Vector::Zero(9);
    EXPECT(assert_equal(zero,a.localCoordinates(a)));
    EXPECT(assert_equal(a,a.retract(zero)));
    Vector9 v = a.localCoordinates(b);
    WorldPoints c  = a.retract(v);
    EXPECT(assert_equal(b,c));
    EXPECT(check_manifold_invariants(a,b));
}

TEST(CameraPoints, manifold){
    BOOST_CONCEPT_ASSERT((internal::HasManifoldPrereqs<CameraPoints>));
    BOOST_CONCEPT_ASSERT((IsManifold<CameraPoints>));
    Point2 centroid(0,0);
    Point2 majAxisTip(1,2);
    Point2 minAxisTip(4,5);
    const CameraPoints a(centroid, majAxisTip, minAxisTip);
    const CameraPoints b(centroid, majAxisTip + Point2(1,2), minAxisTip);
    Vector zero = Vector::Zero(6);
    EXPECT(assert_equal(zero,a.localCoordinates(a)));
    EXPECT(assert_equal(a,a.retract(zero)));
    Vector6 v = a.localCoordinates(b);
    CameraPoints c  = a.retract(v);
    EXPECT(assert_equal(b,c));
    EXPECT(check_manifold_invariants(a,b));
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}
