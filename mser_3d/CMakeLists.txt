# CMake entry point
cmake_minimum_required (VERSION 2.8)
project (mser_3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(GTSAMCMakeTools)
#include(GtsamBuildTypes)
#include(GtsamTesting)
#include(GtsamMatlabWrap)
#find_package(GTSAM REQUIRED) # Uses installed package
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "home/alex/EigenLibrary/cmake")
#set(Eigen3_DIR "home/alex/EigenLibrary/cmake")

#ALEX
#if( CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR )
#    message( FATAL_ERROR "Please select another Build Directory ! (and give it a clever name, like bin_Visual2012_64bits/)" )
#endif()
#if( CMAKE_SOURCE_DIR MATCHES " " )
#	message( "Your Source Directory contains spaces. If you experience problems when compiling, this can be the cause." )
#endif()
#if( CMAKE_BINARY_DIR MATCHES " " )
#	message( "Your Build Directory contains spaces. If you experience problems when compiling, this can be the cause." )
#endif()

# Compile external dependencies 
add_subdirectory (external)

#ALEX
# On Visual 2005 and above, this module can set the debug working directory
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/external/rpavlik-cmake-modules-1c73e35")
#include(CreateLaunchers)
#include(MSVCMultipleProcessCompile) # /MP
#if(INCLUDE_DISTRIB)
#	add_subdirectory(distrib)
#endif(INCLUDE_DISTRIB)

include_directories(
	external/AntTweakBar-1.16/include/
	external/glfw-3.0.3/include/GLFW/
	external/glm-0.9.4.0/
	external/glew-1.9.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
	${Eigen3_INCLUDE_DIRS}
	/usr/include/eigen3
	#${GTSAM_INCLUDE_DIRS}
	.
)

set(ALL_LIBS
	${OPENGL_LIBRARY}
	GLFW_303
	GLEW_190
	${Eigen3_LIBRARIES}
	#gtsam
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
	#tutorial04.cpp
	main.cpp
	#mserClasses.h
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

#find_package(GTSAM REQUIRED) #CRAZY ISSUE adding this along with the other find_package statements breaks GLEW and GLFW linkage....
#include_directories(${GTSAM_INCLUDE_DIRS}) #see above
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(mser_3d
	${ALL_LIBS}
	gtsam
	${OpenCV_LIBS}
)
# Xcode and Visual working directories
#set_target_properties(tutorial04_colored_cube PROPERTIES XCODE_ATTRIBUTE_CONFIGURATION_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tutorial04_colored_cube/")
#create_target_launcher(tutorial04_colored_cube WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/tutorial04_colored_cube/")

SOURCE_GROUP(common REGULAR_EXPRESSION ".*/common/.*" )
SOURCE_GROUP(shaders REGULAR_EXPRESSION ".*/.*shader$" )


#if (NOT ${CMAKE_GENERATOR} MATCHES "Xcode" )
#	add_custom_command(
#	   TARGET mser_3d POST_BUILD
#	   COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_CFG_INTDIR}/tutorial04_colored_cube${CMAKE_EXECUTABLE_SUFFIX}" "${CMAKE_CURRENT_SOURCE_DIR}/tutorial04_colored_cube/"
#	)
#elseif (${CMAKE_GENERATOR} MATCHES "Xcode" )
#endif (NOT ${CMAKE_GENERATOR} MATCHES "Xcode" )

