cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rangeimage)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( SOURCES rangeimage.cpp)
#add library rangeimage
add_library( rangeimage SHARED ${SOURCES} )
add_executable (range_image rangeimage.cpp)
target_link_libraries (rangeimage ${PCL_LIBRARIES})
target_link_libraries(range_image ${PCL_LIBRARIES})
install(TARGETS rangeimage
        # Windows library (*.dll)                                 
        RUNTIME  DESTINATION bin
        # linux shared object (*.so)
        LIBRARY   DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_WRITE GROUP_EXECUTE
                    WORLD_READ WORLD_WRITE WORLD_EXECUTE)
