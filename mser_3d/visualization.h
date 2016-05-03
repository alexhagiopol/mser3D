//
// Created by alex on 10/11/15.
//
#pragma once
#include "InputManager.h"
#include "MserMeasurement.h"
#include "MserObject.h"
#include "MserTrack.h"
#include <gtsam/geometry/SimpleCamera.h>
#include <GL/glew.h> // GLEW
#include <glfw3.h> // GLFW
#include <glm/glm.hpp> //GLM
#include <glm/gtc/matrix_transform.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <math.h>
#include <random>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace glm;
using namespace std;
using namespace gtsam;
#include "common/shader.hpp" //shader.hpp needs the GLM namespace, else you will get "xyz does not name a type" errors.
#include "common/controls.hpp"


std::vector<std::pair<Point3,Point3>> makeRayTracingPairs(std::vector<gtsam::MserTrack>& tracks, std::vector<Pose3>& VOposes);

int drawMserObjects(const InputManager& input, const std::vector<Pose3>& cameraPoses, const std::vector<MserObject>& objects, const std::vector<Vector3>& colors = std::vector<Vector3>(), const std::vector<std::pair<Point3,Point3>>& rays = std::vector<std::pair<Point3,Point3>>());

int drawTexturedMserObjects(const InputManager& input, const std::vector<Pose3>& cameraPoses, const std::vector<MserObject>& objects, const std::vector<Vector3>& colors = std::vector<Vector3>(), const std::vector<std::pair<Point3,Point3>>& rays = std::vector<std::pair<Point3,Point3>>());

void addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(const std::vector<Pose3>& cameraPoses, std::vector<MserObject>& objects, std::vector<Vector3>& colors);

void showTextures();

