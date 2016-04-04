//
// Created by alex on 3/23/16.
//

#include "PointsPose.h"
#include "WorldPoints.h"
#include "CameraPoints.h"
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <CppUnitLite/TestHarness.h>
using namespace gtsam;

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