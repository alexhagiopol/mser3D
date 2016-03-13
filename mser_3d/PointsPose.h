//
// Created by alex on 3/13/16.
//
#pragma once
#include <gtsam/geometry/Point2.h>
#include "MserObject.h"
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam{
    class PointsPose{ //note: Point3s are in *object frame*
    public:
        enum { dimension = 12 };
        Point3 majAxisTip;
        Point3 minAxisTip;
        Pose3 objectPose;
        PointsPose(Point3 majAxisTip_, Point3 minAxisTip_, Pose3 objectPose_){majAxisTip = majAxisTip_; minAxisTip = minAxisTip_; objectPose = objectPose_;}
        //void print(std::string& str){} //TODO make this
        //bool equals(PointsPose& other, double tol = 1e-5){ return other.majAxisTip.equals(majAxisTip,tol) && other.minAxisTip.equals(minAxisTip,tol) && other.objectPose.equals(objectPose,tol);}
        //inline static PointsPose identity() { return PointsPose(Point3(0,0,0),Point3(0,0,0),Pose3()); }
    };
/*
    template<>
    struct traits<PointsPose> : public internal::VectorSpace<PointsPose> {};
    */
} //namespace gtsam

