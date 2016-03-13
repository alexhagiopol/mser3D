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

TEST(measurementFunction, convertObjectToObjectPointsPose){
    Eigen::MatrixXd H1(12,8);
    boost::function<PointsPose(MserObject&)> f = boost::bind(&convertObjectToObjectPointsPose, _1, boost::none);
    //Make object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject object(objectPose,objectAxes);
    PointsPose objectPointsPose = convertObjectToObjectPointsPose(object, H1);
    EXPECT(objectPointsPose.majAxisTip == Point3(3,0,0));
    EXPECT(objectPointsPose.minAxisTip == Point3(0,1,0));
}

int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
}


