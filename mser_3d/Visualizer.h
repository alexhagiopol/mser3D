//
// Created by alex on 10/11/15.
//


#ifndef MSER_3D_VISUALIZER_H
#define MSER_3D_VISUALIZER_H

#include "mserClasses.h"
#include <GL/glew.h> // Include GLEW
#include <glfw3.h> // Include GLFW
#include <glm/glm.hpp> // Include GLM
#include <glm/gtc/matrix_transform.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <random>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <gtsam/geometry/SimpleCamera.h>

//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace glm;
using namespace std;
using namespace gtsam;
#include <common/shader.hpp> //shader.hpp needs the GLM namespace, else you will get "xyz does not name a type" errors.

int produceRandomCubeImages(int numFrames){
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

    //OpenGL setup
    glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); // Random color background
    glEnable(GL_DEPTH_TEST); // Enable depth test
    glDepthFunc(GL_LESS);  // Accept fragment if it closer to the camera than the former one
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    GLuint programID = LoadShaders( "../../TransformVertexShader.vertexshader", "../../ColorFragmentShader.fragmentshader" ); // Create and compile our GLSL program from the shaders
    GLuint MatrixID = glGetUniformLocation(programID, "MVP"); // Get a handle for our "MVP" uniform
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f); // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    // Camera matrix
    glm::mat4 View       = glm::lookAt(
            glm::vec3(4,3,-3), // Camera is at (4,3,-3), in World Space
            glm::vec3(0,0,0), // and looks at the origin
            glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
    );
    glm::mat4 Model      = glm::mat4(1.0f); // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 MVP        = Projection * View * Model; // Our ModelViewProjection : multiplication of our 3 matrices. Remember, matrix multiplication is the other way around

    // Cube vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    GLfloat g_vertex_buffer_data[] = {
            -1.0f,-1.0f,-1.0f, //Triangle 1
            -1.0f,-1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f, 1.0f,-1.0f, //Triangle 2
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
            1.0f,-1.0f, 1.0f, //Triangle 3
            -1.0f,-1.0f,-1.0f,
            1.0f,-1.0f,-1.0f,
            1.0f, 1.0f,-1.0f, //Triangle 4
            1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f,-1.0f,-1.0f, //Triangle 5
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f,-1.0f,
            1.0f,-1.0f, 1.0f, //Triangle 6
            -1.0f,-1.0f, 1.0f,
            -1.0f,-1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f, //Triangle 7
            -1.0f,-1.0f, 1.0f,
            1.0f,-1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, //Triangle 8
            1.0f,-1.0f,-1.0f,
            1.0f, 1.0f,-1.0f,
            1.0f,-1.0f,-1.0f, //Triangle 9
            1.0f, 1.0f, 1.0f,
            1.0f,-1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, //Triangle 10
            1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f,-1.0f,
            1.0f, 1.0f, 1.0f, //Triangle 11
            -1.0f, 1.0f,-1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f, //Triangle 12
            -1.0f, 1.0f, 1.0f,
            1.0f,-1.0f, 1.0f
    };

    //Place vertex info into a buffer
    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    GLuint colorbuffer; //create color buffer outside loop so it can be purged outside loop


    for (int f = 0; f < numFrames; f++){
        // One color for each vertex. They were generated randomly.
        glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); //random background color
        //random cube color
        float cubeR = distr(eng);
        float cubeG = distr(eng);
        float cubeB = distr(eng);

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

        /* //Use this if you need cubes with multicolored faces
        GLfloat g_color_buffer_data[] = {
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
            distr(eng),  distr(eng),  distr(eng),
        };
        */
        //Place color info into a buffer
        glGenBuffers(1, &colorbuffer);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        glUseProgram(programID); // Use our shader
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]); // Send our transformation to the currently bound shader in the "MVP" uniform
        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
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
                (void*)0                          // array buffer offset
        );
        glDrawArrays(GL_TRIANGLES, 0, 12*3); // Draw the triangle ! 12*3 indices starting at 0 -> 12 triangles
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents();
        Mat img(768, 1024, CV_8UC3); //store image data here to output to a file
        glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4); //use fast 4-byte alignment (default anyway) if possible
        glPixelStorei(GL_PACK_ROW_LENGTH, img.step/img.elemSize()); //set length of one complete row in destination data (doesn't need to equal img.cols)
        glReadPixels(0, 0, img.cols, img.rows, GL_BGR, GL_UNSIGNED_BYTE, img.data); //import OpenGL window repsesentation into cv::Mat
        Mat flipped(768, 1024, CV_8UC3); //we have to flip because OpenCV and OpenGL use different xy conventions
        cv::flip(img, flipped, 0);
        char fileName[80];
        sprintf(fileName, "/home/alex/mser/mser_3d/output/img%010i.jpg",f); //format filename
        imwrite(fileName, flipped); //save to output folder
    }
    /*
    // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );
    */
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &colorbuffer);
    glDeleteProgram(programID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
    return 0;
}

int produceMSERMeasurements(std::vector<gtsam::SimpleCamera>& cameras, Point3& target, std::vector<mserMeasurement>& measurements){
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
            mserMeasurement msmt = mserMeasurement(ellipsePose,ellipseAxes);
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

#endif //MSER_3D_VISUALIZER_H
