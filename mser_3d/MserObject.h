//
// Created by alex on 1/11/16.
//
#pragma once
#include <gtsam/base/Manifold.h> //required for MSER object
#include <gtsam/geometry/Pose3.h> //required for MSER object
#include <gtsam/geometry/Pose2.h> //required for MSER measurement
#include <gtsam/geometry/Point2.h> //required for MSER object

using namespace std;

namespace gtsam { //required to keep GCC from complaining
    typedef ProductManifold<Pose3, Point2> MserObject;

    template<>
    struct traits<MserObject> : internal::ManifoldTraits<MserObject> {
        static void Print(const MserObject &o, const string &s = "") {
            cout << s << " (";
            o.first.print();
            cout << ",";
            o.second.print();
            cout << ")" << endl;
            //cout << s << "(" << o.first << "," << o.second << ")" << endl;
        }

        static bool Equals(const MserObject &o1, const MserObject &o2, double tol = 1e-8) {
            return ((o1.first.equals(o2.first, tol)) && (o1.second.equals(o2.second, tol)));
        }
    };
}//namespace gtsam
