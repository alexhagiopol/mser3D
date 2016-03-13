//
// Created by alex on 2/24/16.
//

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testPoint3.cpp
 * @brief  Unit tests for Point3 class
 */
#include "measurementFunction.h"
#include "PointsPose.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Point3)
GTSAM_CONCEPT_LIE_INST(Point3)

static Point3 P(0.2, 0.7, -2);

TEST(Point3, cross) {
    Matrix aH1, aH2;
    boost::function<Point3(const Point3&, const Point3&)> f = boost::bind(&Point3::cross, _1, _2, boost::none, boost::none);
    const Point3 omega(0, 1, 0), theta(4, 6, 8);
    omega.cross(theta, aH1, aH2);
    EXPECT(assert_equal(numericalDerivative21(f, omega, theta), aH1));
    EXPECT(assert_equal(numericalDerivative22(f, omega, theta), aH2));
}

//Tests convertObjectToObjectPointsPose() in measurementFunction.h
TEST(measurementFunction, convertObjectToObjectPointsPose){
    Eigen::MatrixXd H1(12,8);
    boost::function<PointsPose(const MserObject&)> f = boost::bind(&convertObjectToObjectPointsPose, _1, boost::none);
    //Make object
    const Point3 objectCenter(0,0,0);
    const Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    const Point2 objectAxes(3,1);
    const Pose3 objectPose(objectOrientation, objectCenter);
    const MserObject object(objectPose,objectAxes);
    const PointsPose objectPointsPose = convertObjectToObjectPointsPose(object, H1);
    EXPECT(assert_equal(objectPointsPose.majAxisTip, Point3(3,0,0)));
    EXPECT(assert_equal(objectPointsPose.minAxisTip, Point3(0,1,0)));
    EXPECT(assert_equal(objectPointsPose.objectPose, Pose3()));
    //EXPECT(assert_equal(numericalDerivative11(f,object),H1)); //TODO: Ask Frank if I need to make traits for PointsPose to make this work
}

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

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}


