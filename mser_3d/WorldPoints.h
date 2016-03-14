//
// Created by alex on 3/14/16.
//
#pragma once
#include "TripleManifold.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Manifold.h>

using namespace std;
namespace gtsam{
    typedef TripleManifold<Point3, Point3, Point3> WorldPoints;
    template<>
    struct traits<WorldPoints> : internal::ManifoldTraits<WorldPoints> {
        static Point3 minAxisTip(const WorldPoints& p){
            return get<0>(p);
        }
        static Point3 majAxisTip(const WorldPoints& p){
            return get<1>(p);
        }
        static Point3 centroid(const WorldPoints& p){
            return get<2>(p);
        }
        static void Print(const WorldPoints &o, const string &s = "") {
            cout << s << " (";
            get<0>(o).print();
            cout << ",";
            get<1>(o).print();
            cout << ",";
            get<2>(o).print();
            cout << ")" << endl;
        }
        static bool Equals(const WorldPoints &o1, const WorldPoints &o2, double tol = 1e-8) {
            return ((get<0>(o1).equals(get<0>(o2), tol)) && (get<1>(o1).equals(get<1>(o2), tol)) && (get<2>(o1).equals(get<2>(o2), tol)));
        }
    };
}