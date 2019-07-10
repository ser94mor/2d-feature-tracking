cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -Wextra")

project(camera_fusion)

find_package(OpenCV 4.1 REQUIRED)
include_directories(third_parties/catch2-2.7.0)

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_link_libraries (2D_feature_tracking ${OpenCV_LIBRARIES})

add_library(Catch INTERFACE)
add_executable(2D_feature_tracking_test test/data_buffer_test.cpp test/test.cpp)
target_link_libraries(2D_feature_tracking_test Catch)
enable_testing()
add_test( NAME camera_fusion_test COMMAND camera_fusion_test )
