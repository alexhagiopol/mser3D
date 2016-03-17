//
// Created by Alex Hagiopol on 3/17/16.
//

#pragma once
#include "MserObject.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Manifold.h>

namespace gtsam{
    class CameraPoints_{ //note: Point2s are in *object frame*
    public:
        enum { dimension = 6 };
        Point2 majAxisTip_;
        Point2 minAxisTip_;
        Point2 centroid_;
        CameraPoints_(Point2 centroid, Point2 majAxisTip, Point2 minAxisTip):
                centroid_(centroid), majAxisTip_(majAxisTip), minAxisTip_(minAxisTip){}
        void print(const std::string& s)const{
            cout << s << " (";
            centroid_.print();
            cout << ",";
            majAxisTip_.print();
            cout << ",";
            minAxisTip_.print();
            cout << ")" << endl;
        }
        bool equals(const CameraPoints_& other, double tol = 1e-5)const{
            return other.majAxisTip_.equals(majAxisTip_,tol) &&
                   other.minAxisTip_.equals(minAxisTip_,tol) && other.centroid_.equals(centroid_,tol);}
        //inline static CameraPoints_ identity() { return CameraPoints_(Point2(0,0,0),Point2(0,0,0),Pose3()); }
        typedef Eigen::Matrix<double, dimension, 1> TangentVector;
        CameraPoints_ retract(const TangentVector& xi) const {
            Point2 centroid  = centroid_.retract(xi.head<2>());
            Point2 majAxisTip = majAxisTip_ + xi.segment<2>(2);
            Point2 minAxisTip = minAxisTip_ + xi.tail<2>();
            return CameraPoints_(centroid,majAxisTip,minAxisTip);
        }

        /// Compute the coordinates in the tangent space
        TangentVector localCoordinates(const CameraPoints_& other) const {
            TangentVector xi;
            xi.head<2>() = centroid_.localCoordinates(other.centroid_);
            xi.segment<2>(2) = majAxisTip_.localCoordinates(other.majAxisTip_);
            xi.tail<2>() = minAxisTip_.localCoordinates(other.minAxisTip_);
            return xi;
        }
    };

    template<>
    struct traits<CameraPoints_> : public internal::Manifold<CameraPoints_> {};

    template<>
    struct traits<const CameraPoints_> : public internal::Manifold<CameraPoints_> {};
} //namespace gtsam
