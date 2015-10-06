 

#include <GL/glew.h> // Include GLEW
#include <glfw3.h> // Include GLFW
#include <glm/glm.hpp> // Include GLM
#include <glm/gtc/matrix_transform.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/slam/GeneralSFMFactor.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <random>

using namespace std;
using namespace gtsam;
using namespace cv;
using namespace glm;
#include <common/shader.hpp> //shader.hpp needs the GLM namespace, else you will get "xyz does not name a type" errors. That's why we have to #include after declaring the namespace

GLFWwindow* window;

int main() {
	/*
	//Extract VO data from video - save this for later when we make a camera fly through the scene
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
        const Pose3 pose = camera.pose();
        pose.print("Camera pose:\n");
    }
    */

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
	GLuint programID = LoadShaders( "../TransformVertexShader.vertexshader", "../ColorFragmentShader.fragmentshader" ); // Create and compile our GLSL program from the shaders
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
	

	for (int f = 0; f < 5000; f++){	
		// One color for each vertex. They were generated randomly.
		glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); //random background color
		//random cube color
		float randomR = distr(eng);  
		float randomG = distr(eng);
		float randomB = distr(eng);

		GLfloat g_color_buffer_data[] = { 
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
			randomR, randomG, randomB,
		};

		/*
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
		sprintf(fileName, "/home/alex/mser/mser_3d/output2/img%010i.jpg",f); //format filename
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