//
// Created by alex on 3/13/16.
//
#pragma once
#include "MserObject.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam{
    class PointsPose{ //note: Point3s are in *object frame*
    public:
        enum { dimension = 12 };
        Point3 majAxisTip_;
        Point3 minAxisTip_;
        Pose3 objectPose_;
        PointsPose(Point3 majAxisTip, Point3 minAxisTip, Pose3 objectPose):
                majAxisTip_(majAxisTip), minAxisTip_(minAxisTip), objectPose_(objectPose){}
        void print(const std::string& s)const{
            cout << s << " (";
            majAxisTip_.print();
            cout << ",";
            minAxisTip_.print();
            cout << ",";
            objectPose_.print();
            cout << ")" << endl;
        }
        bool equals(const PointsPose& other, double tol = 1e-5)const{
            return other.majAxisTip_.equals(majAxisTip_,tol) &&
                    other.minAxisTip_.equals(minAxisTip_,tol) && other.objectPose_.equals(objectPose_,tol);}
        //inline static PointsPose identity() { return PointsPose(Point3(0,0,0),Point3(0,0,0),Pose3()); }
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;
        PointsPose retract(const TangentVector& xi) const {
            Point3 majAxisTip = majAxisTip_ + xi.head<3>();

            Point3 minAxisTip = minAxisTip_ + xi.segment<3>(3);
            Pose3 objectPose = objectPose_.retract(xi.tail<6>());
            return PointsPose(majAxisTip,minAxisTip,objectPose);
        }

        /// Compute the coordinates in the tangent space
        TangentVector localCoordinates(const PointsPose& other) const {
            TangentVector xi;
            xi.head<3>() = other.majAxisTip_ - majAxisTip_;
            xi.segment<3>(3) = other.minAxisTip_ - minAxisTip_;
            xi.tail<6>() = objectPose_.localCoordinates(other.objectPose_);
            return xi;
        }
    };

    template<>
    struct traits<PointsPose> : public internal::Manifold<PointsPose> {};

    template<>
    struct traits<const PointsPose> : public internal::Manifold<PointsPose> {};
} //namespace gtsam

