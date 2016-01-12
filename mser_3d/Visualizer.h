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

//Draw a virtual box, fly camera around the box, take mser measurements, teturn them
int produceMserMeasurements(const std::vector<gtsam::SimpleCamera>& cameras, Point3& target, std::vector<MserMeasurement>& measurements){
    GLFWwindow* window;
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );
        return -1;
    }

    //Set up window.
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 1024, 768, "MSER 3D", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n" );
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW - OpenGL Extension Wrangler Library
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below

    //Set up random float generator
    random_device rd;
    mt19937 eng(rd());
    uniform_real_distribution<float> distr(0,1);

    //More OpenGL setup
    glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); // Random color background
    glEnable(GL_DEPTH_TEST); // Enable depth test
    glDepthFunc(GL_LESS);  // Accept fragment if it closer to the camera than the former one
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    GLuint programID = LoadShaders( "../../TransformVertexShader.vertexshader", "../../ColorFragmentShader.fragmentshader" ); // Create and compile our GLSL program from the shaders
    GLuint MatrixID = glGetUniformLocation(programID, "MVP"); // Get a handle for our "MVP" uniform
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f); // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units

    GLuint vertexbuffer; //create vertex buffer outside loop so it can be purged outside loop
    GLuint colorbuffer; //create color buffer outside loop so it can be purged outside loop
    for (size_t f = 0; f < cameras.size(); f++) {
        // Camera matrix
        glm::mat4 View = glm::lookAt(
                glm::vec3(cameras[f].pose().x(), cameras[f].pose().y(), cameras[f].pose().z()), // Camera position in world space
                glm::vec3(target.x(), target.y(), target.z()), // and looks at the target
                glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
        );
        glm::mat4 Model = glm::mat4(1.0f); // Model matrix : an identity matrix (model will be at the origin)
        glm::mat4 MVP = Projection * View *
                        Model; // Our ModelViewProjection : multiplication of our 3 matrices. Remember, matrix multiplication is the other way around

        // Cube vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
        // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
        GLfloat g_vertex_buffer_data[] = {
                -1.0f, -1.0f, -1.0f, //Triangle 1
                -1.0f, -1.0f, 1.0f,
                -1.0f, 1.0f, 1.0f,
                1.0f, 1.0f, -1.0f, //Triangle 2
                -1.0f, -1.0f, -1.0f,
                -1.0f, 1.0f, -1.0f,
                1.0f, -1.0f, 1.0f, //Triangle 3
                -1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, 1.0f, -1.0f, //Triangle 4
                1.0f, -1.0f, -1.0f,
                -1.0f, -1.0f, -1.0f,
                -1.0f, -1.0f, -1.0f, //Triangle 5
                -1.0f, 1.0f, 1.0f,
                -1.0f, 1.0f, -1.0f,
                1.0f, -1.0f, 1.0f, //Triangle 6
                -1.0f, -1.0f, 1.0f,
                -1.0f, -1.0f, -1.0f,
                -1.0f, 1.0f, 1.0f, //Triangle 7
                -1.0f, -1.0f, 1.0f,
                1.0f, -1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, //Triangle 8
                1.0f, -1.0f, -1.0f,
                1.0f, 1.0f, -1.0f,
                1.0f, -1.0f, -1.0f, //Triangle 9
                1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, //Triangle 10
                1.0f, 1.0f, -1.0f,
                -1.0f, 1.0f, -1.0f,
                1.0f, 1.0f, 1.0f, //Triangle 11
                -1.0f, 1.0f, -1.0f,
                -1.0f, 1.0f, 1.0f,
                1.0f, 1.0f, 1.0f, //Triangle 12
                -1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, 1.0f
        };

        //Set color of cube
        glClearColor(1.0,1.0,1.0,1.0);
        float cubeR = 0.5;//distr(eng);
        float cubeG = 0.5;//distr(eng);
        float cubeB = 1.0;//distr(eng);
        GLfloat g_color_buffer_data[] = {
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
                cubeR, cubeG, cubeB,
        };

        //Place vertex info into a buffer
        glGenBuffers(1, &vertexbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

        //Place color info into a buffer
        glGenBuffers(1, &colorbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        glUseProgram(programID); // Use our shader
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE,
                           &MVP[0][0]); // Send our transformation to the currently bound shader in the "MVP" uniform
        // 1st attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void *) 0            // array buffer offset
        );
        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                3,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void *) 0                          // array buffer offset
        );
        glDrawArrays(GL_TRIANGLES, 0, 12 * 3); // Draw the triangle ! 12*3 indices starting at 0 -> 12 triangles
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents();
        Mat img(768, 1024, CV_8UC3); //store image data here to output to a file
        glPixelStorei(GL_PACK_ALIGNMENT,
                      (img.step & 3) ? 1 : 4); //use fast 4-byte alignment (default anyway) if possible
        glPixelStorei(GL_PACK_ROW_LENGTH, img.step /
                                          img.elemSize()); //set length of one complete row in destination data (doesn't need to equal img.cols)
        glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE,
                     img.data); //import OpenGL window repsesentation into cv::Mat
        Mat flipped(768, 1024, CV_8UC3); //we have to flip because OpenCV and OpenGL use different xy conventions
        cv::flip(img, flipped, 0);
        char fileName[80];
        sprintf(fileName, "/home/alex/mser/mser_3d/output/original%010lu.jpg", f); //format filename
        imwrite(fileName, flipped); //Use OpenCV to save to output folder

        //Perform MSER with OpenCV
        /*
        int delta = 5; //! delta, in the code, it compares (size_{i}-size_{i-delta})/size_{i-delta}
        int maxArea = 14400; //! prune the area which bigger than maxArea
        int minArea = 60; //! prune the area which smaller than minArea
        float maxVariation = 0.25; //! prune the area have simliar size to its children
        float minDiversity = 0.2; //! trace back to cut off mser with diversity < min_diversity
        The next few params for MSER of color image:
        int maxEvolution = 200; //! for color image, the evolution steps
        double areaThreshold = 1.01; //! the area threshold to cause re-initialize
        double minMargin = 0.003; //! ignore too small margin
        int edgeBlurSize = 5; //! the aperture size for edge blur
         */
   	    int _delta = 5;
        int	_min_area = 6;
        int _max_area = 144000; //tuned to only detect the cube in the scene
        double 	_max_variation = 2500;
        double 	_min_diversity = 0.02;
        int _max_evolution=20000;
        double _area_threshold=1.01;
        double _min_margin=0.00003;
        int _edge_blur_size=50;
        MSER mser(_delta, _min_area, _max_area, _max_variation, _min_diversity, _max_evolution, _area_threshold, _min_margin, _edge_blur_size); //initialize MSER
        vector<vector<Point>> regions; //data structure to store pixels for each region
        /***INSANE OPENCV BEHAVIOR/BUG: if you load an image from a file, MSER works. If you use the raw Mat image, it doesn't work!!! WTF!!!! */
        Mat from_file = imread(fileName,1); //I'm forced to read my matrix from a file
        Mat gray; //apply grayscale
        cvtColor(from_file, gray, CV_BGR2GRAY );

        /* //I tried to debug the OpenCV bug by looking at RGB matrix values. No luck.
        FileStorage from_file_file("/home/alex/mser/mser_3d/from_file.xml", FileStorage::WRITE);
        from_file_file << "FROM_FILE" << from_file;
        FileStorage flipped_file("/home/alex/mser/mser_3d/flipped.xml", FileStorage::WRITE);
        flipped_file << "FROM_OPENGL"<< flipped;
        FileStorage gray_flipped_file("/home/alex/mser/mser_3d/gray_flipped.xml", FileStorage::WRITE);
        gray_flipped_file << "GRAY_FROM_OPENGL"<< gray_flipped;
        */
        const Mat mask;
        mser(gray, regions, mask); //Detect MSER on grayscale image from file. Store results in regions vector.
        //cout << regions.size() << endl;
        for (size_t i = 0; i < regions.size(); i++)
        {
            RotatedRect rr = fitEllipse(regions[i]); //fit ellipse to region. Store info in rotated rectangle data structure
            ellipse(gray, rr, Scalar(0,0,0),5);
            Pose2 ellipsePose = Pose2(rr.center.x,rr.center.y,rr.angle);
            Point2 ellipseAxes = Point2(rr.size.height,rr.size.width);
            MserMeasurement msmt = MserMeasurement(ellipsePose,ellipseAxes);
            measurements.push_back(msmt);
        }
        //imshow("mser",gray_flipped);
        waitKey(1);
        char fileNameMSER[80];
        sprintf(fileNameMSER, "/home/alex/mser/mser_3d/output/mser%010lu.jpg", f); //format filename
        imwrite(fileNameMSER, gray); //Use OpenCV to save to output folder
    }
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    glDeleteProgram(programID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}

int drawMserObjects(const std::vector<MserObject>& objects, const std::vector<Vector3>& colors = std::vector<Vector3>()){
    GLFWwindow *window;
    // Initialise GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    //Set up window.
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1024, 768, "MSER 3D", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW - OpenGL Extension Wrangler Library
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below

    //Set up random float generator
    random_device rd;
    mt19937 eng(0);
    uniform_real_distribution<float> distr(0, 1);

    //More OpenGL setup
    //glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); // Random color background
    glClearColor(1.0, 0.8, 0.8, 0.8); //Set color of background
    glEnable(GL_DEPTH_TEST); // Enable depth test
    glDepthFunc(GL_LESS);  // Accept fragment if it closer to the camera than the former one
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    GLuint programID = LoadShaders("../../TransformVertexShader.vertexshader",
                                   "../../ColorFragmentShader.fragmentshader"); // Create and compile our GLSL program from the shaders
    GLuint MatrixID = glGetUniformLocation(programID, "MVP"); // Get a handle for our "MVP" uniform
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f,
                                            100.0f); // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units

    GLuint vertexbuffer; //create vertex buffer outside loop so it can be purged outside loop
    GLuint colorbuffer; //create color buffer outside loop so it can be purged outside loop

    // Camera matrix
    glm::mat4 View = glm::lookAt(
            glm::vec3(0, 10, 40), //camera position
            glm::vec3(0, 0, 0), // and looks at the target
            glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
    );
    glm::mat4 Model = glm::mat4(1.0f); // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 MVP = Projection * View *
                    Model; // Our ModelViewProjection : multiplication of our 3 matrices. Remember, matrix multiplication is the other way around

    //(objects.size ellipses)(360 triangles / ellipse)(3 points / triangle)(3 doubles / point) + (objects.size axes groups)(3 lines / axis group)(6 doubles / line) + (1 world axis group)(3 lines / world axis group)(6 doubles / world axis line)
    int vertexDataSize = objects.size()*360*3*3 + objects.size()*3*6 + 1*3*6;

    GLfloat g_vertex_buffer_data[vertexDataSize];
    GLfloat g_color_buffer_data[vertexDataSize];

    //Draw 3D ellipses representing objects
    for (int o = 0; o < objects.size(); o++) {
        float cubeR, cubeG, cubeB;
        if (colors.size() == objects.size()){
            cubeR = (float) colors[o][0] / 255;
            cubeG = (float) colors[o][1] / 255;
            cubeB = (float) colors[o][2] / 255;
        } else {
            cubeR = distr(eng);
            cubeG = distr(eng);
            cubeB = distr(eng);
        }
        for (int i = 0 + o*360*9; i < 360*9 + o*360*9; i+=9) {
            // Object vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
            float rad_angle = (i/9) * M_PI / 180;
            float next_rad_angle = (i/9 + 1) * M_PI / 180;
            float ellipse_x_radius = objects[o].second.x();// / 40;
            float ellipse_y_radius = objects[o].second.y();// / 40;
            //cout << ellipse_x_radius << " " << ellipse_y_radius << endl;

            double scaleFactor = 1; //use this to make things further apart, closer together for easier visualization
            Point3 centerInWorldFrame(objects[o].first.x()/scaleFactor,objects[o].first.y()/scaleFactor,objects[o].first.z());
            Point3 axisTipInObjectFrame(cos(rad_angle) * ellipse_x_radius,sin(rad_angle) * ellipse_y_radius,0);
            Point3 nextAxisTipInObjectFrame(cos(next_rad_angle) * ellipse_x_radius,sin(next_rad_angle) * ellipse_y_radius,0);
            Rot3 objectRot = objects[o].first.rotation();
            Point3 axisTipInWorldFrame = objectRot*axisTipInObjectFrame + centerInWorldFrame;
            Point3 nextAxisTipInWorldFrame = objectRot*nextAxisTipInObjectFrame + centerInWorldFrame;

            g_vertex_buffer_data[i] = centerInWorldFrame.x();//objects[o].first.x();
            g_vertex_buffer_data[i+1] = centerInWorldFrame.y();//objects[o].first.y();
            g_vertex_buffer_data[i+2] = centerInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[i+3] = axisTipInWorldFrame.x();//objects[o].first.x() + cos(rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[i+4] = axisTipInWorldFrame.y();//objects[o].first.y() + sin(rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[i+5] = axisTipInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[i+6] = nextAxisTipInWorldFrame.x();//objects[o].first.x() + cos(next_rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[i+7] = nextAxisTipInWorldFrame.y();//objects[o].first.y() + sin(next_rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[i+8] = nextAxisTipInWorldFrame.z();//objects[o].first.z();

            //Set color of object
            g_color_buffer_data[i] = cubeR;
            g_color_buffer_data[i+1] = cubeG;
            g_color_buffer_data[i+2] = cubeB;
            g_color_buffer_data[i+3] = cubeR;
            g_color_buffer_data[i+4] = cubeG;
            g_color_buffer_data[i+5] = cubeB;
            g_color_buffer_data[i+6] = cubeR;
            g_color_buffer_data[i+7] = cubeG;
            g_color_buffer_data[i+8] = cubeB;
        }
    }

    //Draw axes lines in objects' frames. RGB correspond to XYZ.
    int objectAxisLength = 1;
    int vertexDataNum = objects.size()*360*3*3; //start where variable i left off in the previous loop
    for (int o = 0; o < objects.size(); o++) {
        //Object frame:
        Point3 objectCenterInObjectFrame(0,0,0);
        Point3 objectXAxisTipInObjectFrame(objectAxisLength,0,0);
        Point3 objectYAxisTipInObjectFrame(0,objectAxisLength,0);
        Point3 objectZAxisTipInObjectFrame(0,0,objectAxisLength);
        //World frame:
        MserObject object = objects[o];
        Point3 objectCenterInWorldFrame = objectCenterInObjectFrame + object.first.translation();
        Point3 objectXAxisTipInWorldFrame = object.first.rotation()*objectXAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectYAxisTipInWorldFrame = object.first.rotation()*objectYAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectZAxisTipInWorldFrame = object.first.rotation()*objectZAxisTipInObjectFrame + objectCenterInWorldFrame;

        //First line: X axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 0] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 1] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 2] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 3] = objectXAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 4] = objectXAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 5] = objectXAxisTipInWorldFrame.z();
        //Second line: Y axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 6] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 7] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 8] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 9] = objectYAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 10] = objectYAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 11] = objectYAxisTipInWorldFrame.z();
        //Third line: Z axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 12] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 13] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 14] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 15] = objectZAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 16] = objectZAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 17] = objectZAxisTipInWorldFrame.z();

        //First line: X axis
        //First point:
        g_color_buffer_data[vertexDataNum + 0] = 1.0f;
        g_color_buffer_data[vertexDataNum + 1] = 0.0f;
        g_color_buffer_data[vertexDataNum + 2] = 0.0f;
        //Second point:
        g_color_buffer_data[vertexDataNum + 3] = 1.0f;
        g_color_buffer_data[vertexDataNum + 4] = 0.0f;
        g_color_buffer_data[vertexDataNum + 5] = 0.0f;

        //Second line: Y axis
        //First point:
        g_color_buffer_data[vertexDataNum + 6] = 0.0f;
        g_color_buffer_data[vertexDataNum + 7] = 1.0f;
        g_color_buffer_data[vertexDataNum + 8] = 0.0f;
        //Second point
        g_color_buffer_data[vertexDataNum + 9] = 0.0f;
        g_color_buffer_data[vertexDataNum + 10] = 1.0f;
        g_color_buffer_data[vertexDataNum + 11] = 0.0f;

        //Third line: Z axis
        //First point:
        g_color_buffer_data[vertexDataNum + 12] = 0.0f;
        g_color_buffer_data[vertexDataNum + 13] = 0.0f;
        g_color_buffer_data[vertexDataNum + 14] = 1.0f;
        //Second point:
        g_color_buffer_data[vertexDataNum + 15] = 0.0f;
        g_color_buffer_data[vertexDataNum + 16] = 0.0f;
        g_color_buffer_data[vertexDataNum + 17] = 1.0f;

        vertexDataNum += 18; //increment data position
    }

    //Draw lines for world axes. RGB correspond to XYZ
    int worldAxisLength = 10;
    Point3 worldCenter(0,0,0);
    Point3 xAxisTip(worldAxisLength,0,0);
    Point3 yAxisTip(0,worldAxisLength,0);
    Point3 zAxisTip(0,0,worldAxisLength);

    //First line: X axis
    //First point:
    g_vertex_buffer_data[vertexDataSize - 18] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 17] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 16] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[vertexDataSize - 15] = xAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 14] = xAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 13] = xAxisTip.z();
    //Second line: Y axis
    //First point:
    g_vertex_buffer_data[vertexDataSize - 12] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 11] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 10] = worldCenter.z();
    //Second point:
    //First point:
    g_vertex_buffer_data[vertexDataSize - 9] = yAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 8] = yAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 7] = yAxisTip.z();
    //Third line: Z axis
    //First point
    g_vertex_buffer_data[vertexDataSize - 6] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 5] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 4] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[vertexDataSize - 3] = zAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 2] = zAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 1] = zAxisTip.z();

    //First line: X axis
    //First point:
    g_color_buffer_data[vertexDataSize - 18] = 1.0f;
    g_color_buffer_data[vertexDataSize - 17] = 0.0f;
    g_color_buffer_data[vertexDataSize - 16] = 0.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 15] = 1.0f;
    g_color_buffer_data[vertexDataSize - 14] = 0.0f;
    g_color_buffer_data[vertexDataSize - 13] = 0.0f;

    //Second line: Y axis
    //First point:
    g_color_buffer_data[vertexDataSize - 12] = 0.0f;
    g_color_buffer_data[vertexDataSize - 11] = 1.0f;
    g_color_buffer_data[vertexDataSize - 10] = 0.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 9] = 0.0f;
    g_color_buffer_data[vertexDataSize - 8] = 1.0f;
    g_color_buffer_data[vertexDataSize - 7] = 0.0f;

    //Third line: Z axis
    //First point:
    g_color_buffer_data[vertexDataSize - 6] = 0.0f;
    g_color_buffer_data[vertexDataSize - 5] = 0.0f;
    g_color_buffer_data[vertexDataSize - 4] = 1.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 3] = 0.0f;
    g_color_buffer_data[vertexDataSize - 2] = 0.0f;
    g_color_buffer_data[vertexDataSize - 1] = 1.0f;
    // End lines for world axes

    //Place vertex info into a buffer
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    //Place color info into a buffer
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

    //while loop start
    do{
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        glUseProgram(programID); // Use our shader
        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs(window);
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE,&MVP[0][0]); // Send our transformation to the currently bound shader in the "MVP" uniform
        // 1st attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void *) 0            // array buffer offset
        );
        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                3,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void *) 0                          // array buffer offset
        );
        //MOST IMPORTANT DRAWING CODE IS HERE! We take what we stored in the vertex buffer and draw it.
        //Note that here we count VERTICES (e.g. 3 points per triangle or 2 points per line) NOT DOUBLES
        glDrawArrays(GL_TRIANGLES, 0, 360 * 3 * objects.size()); // Draw the 360 triangles per ellipse. Each triangle has 3 points. Hence 360*3*objects.size(). Start at index 0.
        glDrawArrays(GL_LINES,360 * 3 * objects.size(), 360 * 3 * objects.size() + objects.size()*3*2); //Draw object.size # of axes groups. Each axes group has 3 lines. Each line has 2 points.
        glDrawArrays(GL_LINES, 360 * 3 * objects.size() + objects.size()*3*2, 360 * 3 * objects.size() + objects.size()*3*2 + 3*2); //Draw world axes group. Group has 3 lines. Each line has 2 points.
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents();
    }while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );
    //while loop and

    Mat img(768, 1024, CV_8UC3); //store image data here to output to a file
    glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4); //use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize()); //set length of one complete row in destination data (doesn't need to equal img.cols)
    glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data); //import OpenGL window repsesentation into cv::Mat
    Mat flipped(768, 1024, CV_8UC3); //we have to flip because OpenCV and OpenGL use different xy conventions
    cv::flip(img, flipped, 0);
    char fileName[80];
    sprintf(fileName, "/home/alex/mser/mser_3d/output/drawMserObjects.jpg"); //format filename
    imwrite(fileName, flipped); //save to output folder
}

#endif //MSER_3D_VISUALIZER_H
