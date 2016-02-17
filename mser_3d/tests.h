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

//Geometry tests
void testLocateObject();
void testEllipse2DOrientation();

//Visualization tests
void testMserObjectDrawing();
void testPrintSuperimposedMeasurementImages(const InputManager& input);

//MeasurementFunction tests
void testExpressionsOptimization();
void testExpressionsOptimizationWithBadInitialGuess();
void testMeasurementFunction();

//3D Reconstruction tests
void testDisplayPoses(const InputManager& input);
void test3DReconstruction(const InputManager& input);

//test everything
void testAll(const InputManager& input);