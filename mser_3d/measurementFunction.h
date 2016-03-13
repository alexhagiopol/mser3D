//
// Created by Alex Hagiopol on 11/30/15.
//
#pragma once
#include "PointsPose.h"
#include "MserMeasurement.h"
#include "MserObject.h"
#include "geometryFunctions.h"
#include "boost/optional.hpp"
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <math.h>
#include <string>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;

PointsPose convertObjectToObjectPointsPose(const MserObject& object, OptionalJacobian<12,8> Dobject = boost::none);

std::vector<Point3> convertObjectPointsPoseToWorldPoint3s(const PointsPose& objectPointsPose, OptionalJacobian<9,12> Dpointspose = boost::none);

std::vector<Point2> convertWorldPoint3sToCameraPoint2s(const SimpleCamera& camera, std::vector<Point3>& points, OptionalJacobian<6,6> Dpose = boost::none, OptionalJacobian<6,5> Dcal = boost::none, OptionalJacobian<6,9> Dpoints = boost::none);

MserMeasurement convertCameraPoint2sToMeasurement(std::vector<Point2>& cameraPoint2s, OptionalJacobian<5,6> Dpoints = boost::none);

MserMeasurement measurementFunction(const SimpleCamera& camera, const MserObject& object, OptionalJacobian<5,11> Dcamera = boost::none, OptionalJacobian<5,8> Dobject = boost::none);

std::vector<MserMeasurement> createIdealMeasurements(std::vector<SimpleCamera>& cameras, MserObject& object);

typedef Expression<MserObject> MserObject_;
typedef Expression<MserMeasurement> MserMeasurement_;
typedef Expression<SimpleCamera> SimpleCamera_;

inline MserMeasurement_ measurementFunctionExpr(const SimpleCamera_ &camera_, const MserObject_ &object_) {
    MserMeasurement (*f)(const SimpleCamera&, const MserObject&, OptionalJacobian<5,11>, OptionalJacobian<5,8>) = &measurementFunction;
    return MserMeasurement_(f, camera_, object_);
}




