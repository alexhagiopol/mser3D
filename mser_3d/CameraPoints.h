//
// Created by alex on 3/14/16.
//

#pragma once
#include "TripleManifold.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Manifold.h>

using namespace std;
namespace gtsam{
    typedef TripleManifold<Point2, Point2, Point2> CameraPoints;
    template<>
    struct traits<CameraPoints> : internal::ManifoldTraits<CameraPoints> {
        static Point2 minAxisTip(const CameraPoints& p){
            return get<0>(p);
        }
        static Point2 majAxisTip(const CameraPoints& p){
            return get<1>(p);
        }
        static Point2 centroid(const CameraPoints& p){
            return get<2>(p);
        }
        static void Print(const CameraPoints &o, const string &s = "") {
            cout << s << " (";
            get<0>(o).print();
            cout << ",";
            get<1>(o).print();
            cout << ",";
            get<2>(o).print();
            cout << ")" << endl;
        }
        static bool Equals(const CameraPoints &o1, const CameraPoints &o2, double tol = 1e-8) {
            return ((get<0>(o1).equals(get<0>(o2), tol)) && (get<1>(o1).equals(get<1>(o2), tol)) && (get<2>(o1).equals(get<2>(o2), tol)));
        }
    };
}
