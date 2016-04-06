//
// Created by alex on 2/16/16.
//
#pragma once
#include "InputManager.h"
#include "geometryFunctions.h"
#include "measurementFunction.h"
#include "MserTrack.h"
#include "visualization.h"
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/slam/expressions.h>
#include <sys/stat.h> //need for mkdir


//Visualization tests
void testPrintSuperimposedMeasurementImages(const InputManager& input);

//MeasurementFunction tests
void syntheticTestOptimization(bool visualize = false, bool showEachStep = false, bool noisy = false, int levMarIterations = 100);

//3D Reconstruction tests
void realWorldTestOptimization(const InputManager& input, bool showEachStep = false, int levMarIterations = 100);

