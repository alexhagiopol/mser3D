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
    typedef ProductManifold<Pose2, Point2> MserMeasurement;

    template<>
    struct traits<MserMeasurement> : internal::ManifoldTraits<MserMeasurement> {
        static void Print(const MserMeasurement &m, const string &s = "") {
            cout << s << " (";
            m.first.print();
            cout << ",";
            m.second.print();
            cout << ")" << endl;
            //cout << s << "(" << m.first << "," << m.second << ")" << endl;
        }

        static bool Equals(const MserMeasurement &m1, const MserMeasurement &m2, double tol = 1e-8) {
            return ((m1.first.equals(m2.first, tol)) && (m1.second.equals(m2.second, tol)));
        }
    };
} //namespace gtsam


