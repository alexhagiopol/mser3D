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
        Point3 majAxisTip;
        Point3 minAxisTip;
        Pose3 objectPose;
        PointsPose(Point3 majAxisTip_, Point3 minAxisTip_, Pose3 objectPose_){majAxisTip = majAxisTip_; minAxisTip = minAxisTip_; objectPose = objectPose_;}
    };
} //namespace gtsam

