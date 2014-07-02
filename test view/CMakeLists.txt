cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visualizer)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( SOURCES visualizer.cpp)
add_executable (visualizer visualizer.cpp)
target_link_libraries (visualizer ${PCL_LIBRARIES})


