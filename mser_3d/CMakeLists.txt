# CMake entry point
cmake_minimum_required (VERSION 2.8)
project (mser_3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

# Compile external dependencies 
add_subdirectory (external)

include_directories(
	external/AntTweakBar-1.16/include/
	external/glfw-3.0.3/include/GLFW/
	external/glm-0.9.4.0/
	external/glew-1.9.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
	${Eigen3_INCLUDE_DIRS}
	/usr/include/eigen3
	.
)

set(ALL_LIBS
	${OPENGL_LIBRARY}
	GLFW_303
	GLEW_190
	${Eigen3_LIBRARIES}
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)
set(OpenCV_DIR "/home/alex/opencv-2.4.11/build")
set(OpenCV_FOUND 1) #fixes issue in CLion that prevents building
find_package(GTSAM REQUIRED) #CRAZY ISSUE adding this along with the other find_package statements breaks GLEW and GLFW linkage....
include_directories(${GTSAM_INCLUDE_DIRS}) #see above
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Tutorial 4
add_executable(mser_3d
	main.cpp
	Visualizer.h
	geometryFunctions.h
	measurementFunction.h
	testMeasurementFunction.h
	common/shader.cpp
	common/shader.hpp
	common/controls.cpp
	common/controls.hpp
	TransformVertexShader.vertexshader
	ColorFragmentShader.fragmentshader
	Visualizer.cpp
	MserObject.h
	MserMeasurement.h
	MserTrack.h)

target_link_libraries(mser_3d
	${ALL_LIBS}
	gtsam
	${OpenCV_LIBS}
)

SOURCE_GROUP(common REGULAR_EXPRESSION ".*/common/.*" )
SOURCE_GROUP(shaders REGULAR_EXPRESSION ".*/.*shader$" )


