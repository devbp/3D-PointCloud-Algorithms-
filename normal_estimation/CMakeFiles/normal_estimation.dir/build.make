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
CMAKE_SOURCE_DIR = "/home/poudel/Point cloud codes/normal_estimation"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/poudel/Point cloud codes/normal_estimation"

# Include any dependencies generated for this target.
include CMakeFiles/normal_estimation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/normal_estimation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/normal_estimation.dir/flags.make

CMakeFiles/normal_estimation.dir/normalestimation.cpp.o: CMakeFiles/normal_estimation.dir/flags.make
CMakeFiles/normal_estimation.dir/normalestimation.cpp.o: normalestimation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report "/home/poudel/Point cloud codes/normal_estimation/CMakeFiles" $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/normal_estimation.dir/normalestimation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/normal_estimation.dir/normalestimation.cpp.o -c "/home/poudel/Point cloud codes/normal_estimation/normalestimation.cpp"

CMakeFiles/normal_estimation.dir/normalestimation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/normal_estimation.dir/normalestimation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E "/home/poudel/Point cloud codes/normal_estimation/normalestimation.cpp" > CMakeFiles/normal_estimation.dir/normalestimation.cpp.i

CMakeFiles/normal_estimation.dir/normalestimation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/normal_estimation.dir/normalestimation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S "/home/poudel/Point cloud codes/normal_estimation/normalestimation.cpp" -o CMakeFiles/normal_estimation.dir/normalestimation.cpp.s

CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.requires:
.PHONY : CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.requires

CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.provides: CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.requires
	$(MAKE) -f CMakeFiles/normal_estimation.dir/build.make CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.provides.build
.PHONY : CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.provides

CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.provides.build: CMakeFiles/normal_estimation.dir/normalestimation.cpp.o

# Object files for target normal_estimation
normal_estimation_OBJECTS = \
"CMakeFiles/normal_estimation.dir/normalestimation.cpp.o"

# External object files for target normal_estimation
normal_estimation_EXTERNAL_OBJECTS =

normal_estimation: CMakeFiles/normal_estimation.dir/normalestimation.cpp.o
normal_estimation: CMakeFiles/normal_estimation.dir/build.make
normal_estimation: /usr/lib/libboost_system-mt.so
normal_estimation: /usr/lib/libboost_filesystem-mt.so
normal_estimation: /usr/lib/libboost_thread-mt.so
normal_estimation: /usr/lib/libboost_date_time-mt.so
normal_estimation: /usr/lib/libboost_iostreams-mt.so
normal_estimation: /usr/local/lib/libpcl_common.so
normal_estimation: /usr/lib/libflann_cpp_s.a
normal_estimation: /usr/local/lib/libpcl_kdtree.so
normal_estimation: /usr/local/lib/libpcl_octree.so
normal_estimation: /usr/local/lib/libpcl_search.so
normal_estimation: /usr/lib/libvtkCommon.so.5.8.0
normal_estimation: /usr/lib/libvtkRendering.so.5.8.0
normal_estimation: /usr/lib/libvtkHybrid.so.5.8.0
normal_estimation: /usr/local/lib/libpcl_io.so
normal_estimation: /usr/local/lib/libpcl_features.so
normal_estimation: /usr/local/lib/libpcl_sample_consensus.so
normal_estimation: /usr/local/lib/libpcl_filters.so
normal_estimation: /usr/local/lib/libpcl_keypoints.so
normal_estimation: /usr/local/lib/libpcl_segmentation.so
normal_estimation: /usr/lib/libqhull.so
normal_estimation: /usr/local/lib/libpcl_surface.so
normal_estimation: /usr/local/lib/libpcl_visualization.so
normal_estimation: /usr/local/lib/libpcl_registration.so
normal_estimation: /usr/local/lib/libpcl_tracking.so
normal_estimation: /usr/lib/libvtkParallel.so.5.8.0
normal_estimation: /usr/lib/libvtkRendering.so.5.8.0
normal_estimation: /usr/lib/libvtkGraphics.so.5.8.0
normal_estimation: /usr/lib/libvtkImaging.so.5.8.0
normal_estimation: /usr/lib/libvtkIO.so.5.8.0
normal_estimation: /usr/lib/libvtkFiltering.so.5.8.0
normal_estimation: /usr/lib/libvtkCommon.so.5.8.0
normal_estimation: /usr/lib/libvtksys.so.5.8.0
normal_estimation: CMakeFiles/normal_estimation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable normal_estimation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/normal_estimation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/normal_estimation.dir/build: normal_estimation
.PHONY : CMakeFiles/normal_estimation.dir/build

CMakeFiles/normal_estimation.dir/requires: CMakeFiles/normal_estimation.dir/normalestimation.cpp.o.requires
.PHONY : CMakeFiles/normal_estimation.dir/requires

CMakeFiles/normal_estimation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/normal_estimation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/normal_estimation.dir/clean

CMakeFiles/normal_estimation.dir/depend:
	cd "/home/poudel/Point cloud codes/normal_estimation" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/poudel/Point cloud codes/normal_estimation" "/home/poudel/Point cloud codes/normal_estimation" "/home/poudel/Point cloud codes/normal_estimation" "/home/poudel/Point cloud codes/normal_estimation" "/home/poudel/Point cloud codes/normal_estimation/CMakeFiles/normal_estimation.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/normal_estimation.dir/depend

