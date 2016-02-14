//
// Created by alex on 10/11/15.
//


#ifndef MSER_3D_VISUALIZER_H
#define MSER_3D_VISUALIZER_H

#include "MserMeasurement.h"
#include "MserObject.h"

#include <gtsam/geometry/SimpleCamera.h>

#include <GL/glew.h> // Include GLEW
#include <glfw3.h> // Include GLFW
#include <glm/glm.hpp> // Include GLM
#include <glm/gtc/matrix_transform.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"

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

class Visualizer{
public:
    Visualizer(){}
    int produceMserMeasurements(const std::vector<gtsam::SimpleCamera>& cameras, Point3& target, std::vector<MserMeasurement>& measurements);
    int drawMserObjects(const std::vector<MserObject>& objects, const std::vector<Vector3>& colors = std::vector<Vector3>());
};
//Draw a virtual box, fly camera around the box, take mser measurements, teturn them


#endif //MSER_3D_VISUALIZER_H
