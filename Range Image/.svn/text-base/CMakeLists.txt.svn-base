cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(rangeimage)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( SOURCES rangeimage.cpp)
add_executable (rangeimage rangeimage.cpp)
target_link_libraries (rangeimage ${PCL_LIBRARIES})


