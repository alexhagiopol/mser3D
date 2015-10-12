cmake_minimum_required(VERSION 2.8)
project(mser_3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(SOURCE_FILES main.cpp
                 /home/alex/mser/mser_3d/common/shader.cpp
	         /home/alex/mser/mser_3d/common/shader.hpp
		 TransformVertexShader.vertexshader
	         ColorFragmentShader.fragmentshader
		Visualizer.h)
set(OpenCV_DIR "/home/alex/opencv-2.4.11/build")

#Include OpenGL
find_package(OpenGL REQUIRED)

#Include OpenCV
find_package(OpenCV REQUIRED)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# Compile external dependencies 
add_subdirectory (/home/alex/mser/mser_3d/external)

#These assist OpenGL
include_directories(
	/home/alex/mser/mser_3d/external/AntTweakBar-1.16/include/
	/home/alex/mser/mser_3d/external/glfw-3.0.3/include/GLFW/
	/home/alex/mser/mser_3d/external/glm-0.9.4.0/
	/home/alex/mser/mser_3d/external/glew-1.9.0/include/
	/home/alex/mser/mser_3d/external/assimp-3.0.1270/include/
	/home/alex/mser/mser_3d/external/bullet-2.81-rev2613/src/
	.
)

#OpenGL et al
set(OPENGL_STUFF
	${OPENGL_LIBRARY}
	GLFW_303
	GLEW_190
)

#More settings for OpenGL
add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(mser_3d ${SOURCE_FILES})
target_link_libraries(mser_3d gtsam ${OPENGL_STUFF} ${OpenCV_LIBS})
