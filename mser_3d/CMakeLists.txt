# CMake entry point
cmake_minimum_required (VERSION 2.8)
project (mser_3d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(APPLE)
	include_directories(
		/usr/local/include/eigen3
		/Users/alexhagiopol/gtsam
		.
	)
	set(OpenCV_DIR "/home/alex/opencv-2.4.11/build")
	set(OpenCV_FOUND 1) #fixes issue in CLion that prevents building
	find_package(GTSAM REQUIRED) #CRAZY ISSUE adding this along with the other find_package statements breaks GLEW and GLFW linkage....
	include_directories(${GTSAM_INCLUDE_DIRS}) #see above
	find_package(OpenCV REQUIRED)
	include_directories(${OpenCV_INCLUDE_DIRS})
	add_executable(testManifolds
			${PROJECT_SOURCE_DIR}/CppUnitLite/Failure.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestHarness.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.cpp
			testsManifolds.cpp
			geometryFunctions.h
			geometryFunctions.cpp
			measurementFunction.h
			measurementFunction.cpp
			MserObject.h
			MserMeasurement.h
			MserTrack.h
			optimization.h
			optimization.cpp
			PointsPose.h
			WorldPoints.h
			CameraPoints.h
			TripleManifold.h
	)
	target_link_libraries(testManifolds
			${ALL_LIBS}
			gtsam
			${OpenCV_LIBS}
	)
else()
	find_package(OpenGL REQUIRED)

	#Compile external dependencies
	add_subdirectory (external)
	include_directories(
			external/AntTweakBar-1.16/include/
			external/glfw-3.0.3/include/GLFW/
			external/glm-0.9.4.0/
			external/glew-1.9.0/include/
			external/assimp-3.0.1270/include/
			external/bullet-2.81-rev2613/src/
			/usr/include/eigen3
			/home/gtsam
			.
	)
	set(ALL_LIBS
			${OPENGL_LIBRARY}
			GLFW_303
			GLEW_190
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

	add_executable(testManifolds
			${PROJECT_SOURCE_DIR}/CppUnitLite/Failure.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestHarness.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.cpp
			testManifolds.cpp
			MserMeasurement.h
			MserTrack.h
			PointsPose.h
			WorldPoints.h
			CameraPoints.h
			TripleManifold.h
			)

	add_executable(testMeasurementFunction
			${PROJECT_SOURCE_DIR}/CppUnitLite/Failure.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/Test.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestHarness.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestRegistry.cpp
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.h
			${PROJECT_SOURCE_DIR}/CppUnitLite/TestResult.cpp
			testMeasurementFunction.cpp
			measurementFunction.h
			measurementFunction.cpp
			MserObject.h
			MserMeasurement.h
			MserTrack.h
			optimization.h
			optimization.cpp
			PointsPose.h
			WorldPoints.h
			CameraPoints.h
			TripleManifold.h
			)
	add_executable(mser_3d
			main.cpp
			visualization.h
			visualization.cpp
			geometryFunctions.h
			geometryFunctions.cpp
			measurementFunction.h
			measurementFunction.cpp
			common/shader.cpp
			common/shader.hpp
			common/controls.cpp
			common/controls.hpp
			TransformVertexShader.vertexshader
			ColorFragmentShader.fragmentshader
			MserObject.h
			MserMeasurement.h
			MserTrack.h
			oldTests.h
			oldTests.cpp
			optimization.h
			optimization.cpp
			TripleManifold.h
			PointsPose.h
			WorldPoints.h
			CameraPoints.h
			InputManager.cpp
			InputManager.h Tracker.cpp Tracker.h Frame.cpp Frame.h MserMeasurementColor.cpp MserMeasurementColor.h)

	target_link_libraries(mser_3d
			${ALL_LIBS}
			gtsam
			${OpenCV_LIBS}
			)

	target_link_libraries(testManifolds
			${ALL_LIBS}
			gtsam
			${OpenCV_LIBS}
			)

	target_link_libraries(testMeasurementFunction
			${ALL_LIBS}
			gtsam
			${OpenCV_LIBS}
			)

	SOURCE_GROUP(common REGULAR_EXPRESSION ".*/common/.*" )
	SOURCE_GROUP(shaders REGULAR_EXPRESSION ".*/.*shader$" )
endif()