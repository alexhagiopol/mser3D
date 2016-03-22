//
// Created by Alex Hagiopol on 11/30/15.
//
#pragma once
#include "PointsPose.h"
#include "WorldPoints.h"
#include "CameraPoints.h"
#include "MserMeasurement.h"
#include "MserObject.h"
#include "boost/optional.hpp"
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/slam/expressions.h>  //required for optimization with Expressions syntax
#include <math.h>
#include <string>
#include <random>
#include <vector>
#include <limits>
#include <iostream>

using namespace gtsam;
using namespace std;

static const double inf = std::numeric_limits<double>::infinity();

PointsPose convertObjectToPointsPose(const MserObject& object, OptionalJacobian<12,8> Dobject = boost::none);

WorldPoints convertPointsPoseToWorldPoints(const PointsPose& objectPointsPose, OptionalJacobian<9,12> Dpointspose = boost::none);

CameraPoints convertWorldPointsToCameraPoints(const SimpleCamera& camera, const WorldPoints& points, OptionalJacobian<6,11> Dcamera = boost::none, OptionalJacobian<6,9> Dpoints = boost::none);

//Returns orientation of 2D ellipse given center point and major axis point
double ellipse2DOrientation(const Point2& center, const Point2& majorAxisPoint, OptionalJacobian<1,2> Dcenter = boost::none, OptionalJacobian<1,2> Dmajaxis = boost::none);

MserMeasurement convertCameraPointsToMeasurement(const CameraPoints& cameraPoints, OptionalJacobian<5,6> Dpoints = boost::none);

MserMeasurement measurementFunction(const SimpleCamera& camera, const MserObject& object, OptionalJacobian<5,11> Dcamera = boost::none, OptionalJacobian<5,8> Dobject = boost::none);

std::vector<MserMeasurement> createIdealMeasurements(const std::vector<SimpleCamera>& cameras, MserObject& object);

Pose2 toyExperiment(const Point2& center, const double& theta, OptionalJacobian<3,2> Dcenter = boost::none, OptionalJacobian<3,1> Dtheta = boost::none);

typedef Expression<MserObject> MserObject_;
typedef Expression<MserMeasurement> MserMeasurement_;
typedef Expression<SimpleCamera> SimpleCamera_;

inline MserMeasurement_ measurementFunctionExpr(const SimpleCamera_ &camera_, const MserObject_ &object_) {
    MserMeasurement (*f)(const SimpleCamera&, const MserObject&, OptionalJacobian<5,11>, OptionalJacobian<5,8>) = &measurementFunction;
    return MserMeasurement_(f, camera_, object_);
}




