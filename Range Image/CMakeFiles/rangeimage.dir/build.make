# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/poudel/Point cloud codes/Range Image"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/poudel/Point cloud codes/Range Image"

# Include any dependencies generated for this target.
include CMakeFiles/rangeimage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rangeimage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rangeimage.dir/flags.make

CMakeFiles/rangeimage.dir/rangeimage.cpp.o: CMakeFiles/rangeimage.dir/flags.make
CMakeFiles/rangeimage.dir/rangeimage.cpp.o: rangeimage.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/poudel/Point cloud codes/Range Image/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/rangeimage.dir/rangeimage.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/rangeimage.dir/rangeimage.cpp.o -c "/home/poudel/Point cloud codes/Range Image/rangeimage.cpp"

CMakeFiles/rangeimage.dir/rangeimage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rangeimage.dir/rangeimage.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/poudel/Point cloud codes/Range Image/rangeimage.cpp" > CMakeFiles/rangeimage.dir/rangeimage.cpp.i

CMakeFiles/rangeimage.dir/rangeimage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rangeimage.dir/rangeimage.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/poudel/Point cloud codes/Range Image/rangeimage.cpp" -o CMakeFiles/rangeimage.dir/rangeimage.cpp.s

CMakeFiles/rangeimage.dir/rangeimage.cpp.o.requires:
.PHONY : CMakeFiles/rangeimage.dir/rangeimage.cpp.o.requires

CMakeFiles/rangeimage.dir/rangeimage.cpp.o.provides: CMakeFiles/rangeimage.dir/rangeimage.cpp.o.requires
	$(MAKE) -f CMakeFiles/rangeimage.dir/build.make CMakeFiles/rangeimage.dir/rangeimage.cpp.o.provides.build
.PHONY : CMakeFiles/rangeimage.dir/rangeimage.cpp.o.provides

CMakeFiles/rangeimage.dir/rangeimage.cpp.o.provides.build: CMakeFiles/rangeimage.dir/rangeimage.cpp.o

# Object files for target rangeimage
rangeimage_OBJECTS = \
"CMakeFiles/rangeimage.dir/rangeimage.cpp.o"

# External object files for target rangeimage
rangeimage_EXTERNAL_OBJECTS =

rangeimage: CMakeFiles/rangeimage.dir/rangeimage.cpp.o
rangeimage: CMakeFiles/rangeimage.dir/build.make
rangeimage: /usr/lib/libboost_system-mt.so
rangeimage: /usr/lib/libboost_filesystem-mt.so
rangeimage: /usr/lib/libboost_thread-mt.so
rangeimage: /usr/lib/libboost_date_time-mt.so
rangeimage: /usr/lib/libboost_iostreams-mt.so
rangeimage: /usr/lib/libboost_mpi-mt.so
rangeimage: /usr/lib/libboost_serialization-mt.so
rangeimage: /usr/local/lib/libpcl_common.so
rangeimage: /usr/lib/libflann_cpp_s.a
rangeimage: /usr/local/lib/libpcl_kdtree.so
rangeimage: /usr/local/lib/libpcl_octree.so
rangeimage: /usr/local/lib/libpcl_search.so
rangeimage: /usr/local/lib/libpcl_sample_consensus.so
rangeimage: /usr/local/lib/libpcl_filters.so
rangeimage: /usr/lib/libOpenNI.so
rangeimage: /usr/lib/libvtkCommon.so.5.8.0
rangeimage: /usr/lib/libvtkRendering.so.5.8.0
rangeimage: /usr/lib/libvtkHybrid.so.5.8.0
rangeimage: /usr/lib/libvtkCharts.so.5.8.0
rangeimage: /usr/local/lib/libpcl_io.so
rangeimage: /usr/local/lib/libpcl_features.so
rangeimage: /usr/local/lib/libpcl_visualization.so
rangeimage: /usr/local/lib/libpcl_ml.so
rangeimage: /usr/local/lib/libpcl_segmentation.so
rangeimage: /usr/local/lib/libpcl_people.so
rangeimage: /usr/local/lib/libpcl_keypoints.so
rangeimage: /usr/local/lib/libpcl_outofcore.so
rangeimage: /usr/local/lib/libpcl_stereo.so
rangeimage: /usr/lib/libqhull.so
rangeimage: /usr/local/lib/libpcl_surface.so
rangeimage: /usr/local/lib/libpcl_registration.so
rangeimage: /usr/local/lib/libpcl_recognition.so
rangeimage: /usr/local/lib/libpcl_tracking.so
rangeimage: /usr/lib/libboost_system-mt.so
rangeimage: /usr/lib/libboost_filesystem-mt.so
rangeimage: /usr/lib/libboost_thread-mt.so
rangeimage: /usr/lib/libboost_date_time-mt.so
rangeimage: /usr/lib/libboost_iostreams-mt.so
rangeimage: /usr/lib/libboost_mpi-mt.so
rangeimage: /usr/lib/libboost_serialization-mt.so
rangeimage: /usr/lib/libqhull.so
rangeimage: /usr/lib/libOpenNI.so
rangeimage: /usr/lib/libflann_cpp_s.a
rangeimage: /usr/lib/libvtkCommon.so.5.8.0
rangeimage: /usr/lib/libvtkRendering.so.5.8.0
rangeimage: /usr/lib/libvtkHybrid.so.5.8.0
rangeimage: /usr/lib/libvtkCharts.so.5.8.0
rangeimage: /usr/local/lib/libpcl_common.so
rangeimage: /usr/local/lib/libpcl_kdtree.so
rangeimage: /usr/local/lib/libpcl_octree.so
rangeimage: /usr/local/lib/libpcl_search.so
rangeimage: /usr/local/lib/libpcl_sample_consensus.so
rangeimage: /usr/local/lib/libpcl_filters.so
rangeimage: /usr/local/lib/libpcl_io.so
rangeimage: /usr/local/lib/libpcl_features.so
rangeimage: /usr/local/lib/libpcl_visualization.so
rangeimage: /usr/local/lib/libpcl_ml.so
rangeimage: /usr/local/lib/libpcl_segmentation.so
rangeimage: /usr/local/lib/libpcl_people.so
rangeimage: /usr/local/lib/libpcl_keypoints.so
rangeimage: /usr/local/lib/libpcl_outofcore.so
rangeimage: /usr/local/lib/libpcl_stereo.so
rangeimage: /usr/local/lib/libpcl_surface.so
rangeimage: /usr/local/lib/libpcl_registration.so
rangeimage: /usr/local/lib/libpcl_recognition.so
rangeimage: /usr/local/lib/libpcl_tracking.so
rangeimage: /usr/lib/libvtkViews.so.5.8.0
rangeimage: /usr/lib/libvtkInfovis.so.5.8.0
rangeimage: /usr/lib/libvtkWidgets.so.5.8.0
rangeimage: /usr/lib/libvtkHybrid.so.5.8.0
rangeimage: /usr/lib/libvtkParallel.so.5.8.0
rangeimage: /usr/lib/libvtkVolumeRendering.so.5.8.0
rangeimage: /usr/lib/libvtkRendering.so.5.8.0
rangeimage: /usr/lib/libvtkGraphics.so.5.8.0
rangeimage: /usr/lib/libvtkImaging.so.5.8.0
rangeimage: /usr/lib/libvtkIO.so.5.8.0
rangeimage: /usr/lib/libvtkFiltering.so.5.8.0
rangeimage: /usr/lib/libvtkCommon.so.5.8.0
rangeimage: /usr/lib/libvtksys.so.5.8.0
rangeimage: CMakeFiles/rangeimage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable rangeimage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rangeimage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rangeimage.dir/build: rangeimage
.PHONY : CMakeFiles/rangeimage.dir/build

CMakeFiles/rangeimage.dir/requires: CMakeFiles/rangeimage.dir/rangeimage.cpp.o.requires
.PHONY : CMakeFiles/rangeimage.dir/requires

CMakeFiles/rangeimage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rangeimage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rangeimage.dir/clean

CMakeFiles/rangeimage.dir/depend:
	cd "/home/poudel/Point cloud codes/Range Image" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/poudel/Point cloud codes/Range Image" "/home/poudel/Point cloud codes/Range Image" "/home/poudel/Point cloud codes/Range Image" "/home/poudel/Point cloud codes/Range Image" "/home/poudel/Point cloud codes/Range Image/CMakeFiles/rangeimage.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/rangeimage.dir/depend

