//
// Created by alex on 3/13/16.
//
#pragma once
#include "MserObject.h"
#include "TripleManifold.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam{
    typedef TripleManifold<Pose3, Point3, Point3> PointsPose;
    template<>
    struct traits<PointsPose> : internal::ManifoldTraits<PointsPose> {
        static Pose3 objectPose(const PointsPose& p){
            return get<0>(p);
        }
        static Point3 majAxisTip(const PointsPose& p){
            return get<1>(p);
        }
        static Point3 minAxisTip(const PointsPose& p){
            return get<2>(p);
        }
        static void Print(const PointsPose &o, const string &s = "") {
            cout << s << " (";
            get<0>(o).print();
            cout << ",";
            get<1>(o).print();
            cout << ",";
            get<2>(o).print();
            cout << ")" << endl;
        }
        static bool Equals(const PointsPose &o1, const PointsPose &o2, double tol = 1e-8) {
            return ((get<0>(o1).equals(get<0>(o2), tol)) && (get<1>(o1).equals(get<1>(o2), tol)) && (get<2>(o1).equals(get<2>(o2), tol)));
        }
    };
}