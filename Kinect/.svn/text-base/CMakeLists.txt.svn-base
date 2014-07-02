cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(kinect_PCL_viewer)
 
find_package(PCL 1.6 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB kinectpclviewer_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(kinectPCLviewer ${kinectpclviewer_SRC})
 
target_link_libraries (kinectPCLviewer ${PCL_LIBRARIES})
