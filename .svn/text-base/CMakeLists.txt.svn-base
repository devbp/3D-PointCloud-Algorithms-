cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( SOURCES displaypointcloud.cpp)
#add library PointCloud
add_library( PointCloud SHARED ${SOURCES} )
add_executable (cloud_viewer displaypointcloud.cpp)
target_link_libraries (PointCloud ${PCL_LIBRARIES})
target_link_libraries(cloud_viewer ${PCL_LIBRARIES})
install(TARGETS PointCloud
        # Windows library (*.dll)                                 
        RUNTIME  DESTINATION bin
        # linux shared object (*.so)
        LIBRARY   DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_WRITE GROUP_EXECUTE
                    WORLD_READ WORLD_WRITE WORLD_EXECUTE)
