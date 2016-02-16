//
// Created by alex on 11/9/15.
//
#pragma once
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>  //required for optimization with Expressions syntax
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include "boost/optional.hpp"

#include <iostream>
#include <math.h>
#include <random>
#include <string>
#include <vector>




std::vector<gtsam::Pose3> alexCreatePoses(double radius, gtsam::Point3 target, int numPoses);

std::vector<gtsam::SimpleCamera> alexCreateCameras(double radius, gtsam::Point3 target, int numCams);

void printJingData();

/* ************************************************************************* */
//Compatible with GTSAM example code. Prefer using this over including Dataset.h
std::vector<gtsam::Point3> createPoints();

/* ************************************************************************* */
//Compatible with GTSAM example code. Prefer using this over including Dataset.h
std::vector<gtsam::Pose3> createPoses();

//Example optimization using 3D points. Given a set of data points
gtsam::Values locateObject(gtsam::Point3 target, gtsam::Point3 guess, int numCams, double radius);

//Returns orientation of 2D ellipse given center point and major axis point
double ellipse2DOrientation(gtsam::Point2& center, gtsam::Point2& majorAxisPoint, gtsam::OptionalJacobian<1,2> H1 = boost::none, gtsam::OptionalJacobian<1,2> H2 = boost::none);
