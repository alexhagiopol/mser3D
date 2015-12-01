//
// Created by alex on 11/9/15.
//

#ifndef MSER_3D_MSERCLASSES_H
#define MSER_3D_MSERCLASSES_H

#include <gtsam/base/Manifold.h> //required for MSER object
#include <gtsam/geometry/Pose3.h> //required for MSER object
#include <gtsam/geometry/Pose2.h> //required for MSER measurement
#include <gtsam/geometry/Point2.h> //required for MSER object

using namespace std;

namespace gtsam { //required to keep GCC from complaining
    typedef ProductManifold<Pose3, Point2> mserObject;

    template<>
    struct traits<mserObject> : internal::ManifoldTraits<mserObject> {
        static void Print(const mserObject &o, const string &s = "") {
            cout << s << " (";
            o.first.print();
            cout << ",";
            o.second.print();
            cout << ")" << endl;
            //cout << s << "(" << o.first << "," << o.second << ")" << endl;
        }

        static bool Equals(const mserObject &o1, const mserObject &o2, double tol = 1e-8) {
            return ((o1.first.equals(o2.first, tol)) && (o1.second.equals(o2.second, tol)));
        }
    };

    typedef ProductManifold<Pose2, Point2> mserMeasurement;

    template<>
    struct traits<mserMeasurement> : internal::ManifoldTraits<mserMeasurement> {
        static void Print(const mserMeasurement &m, const string &s = "") {
            cout << s << " (";
            m.first.print();
            cout << ",";
            m.second.print();
            cout << ")" << endl;
            //cout << s << "(" << m.first << "," << m.second << ")" << endl;
        }

        static bool Equals(const mserMeasurement &m1, const mserMeasurement &m2, double tol = 1e-8) {
            return ((m1.first.equals(m2.first, tol)) && (m1.second.equals(m2.second, tol)));
        }
    };

    typedef ProductManifold<Point2,Point2> MyPoint2Pair;

    // Define any direct product group to be a model of the multiplicative Group concept
    template<> struct traits<MyPoint2Pair> : internal::ManifoldTraits<MyPoint2Pair> {
        static void Print(const MyPoint2Pair& m, const string& s = "") {
            cout << s << "(" << m.first << "," << m.second << ")" << endl;
        }
        static bool Equals(const MyPoint2Pair& m1, const MyPoint2Pair& m2, double tol = 1e-8) {
            return m1 == m2;
        }
    };
}//namespace gtsam

#endif //MSER_3D_MSERCLASSES_H
