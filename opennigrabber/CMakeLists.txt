cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_camera)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinectgrabber colorggrabber.cpp)
target_link_libraries (kinectgrabber ${PCL_LIBRARIES})
