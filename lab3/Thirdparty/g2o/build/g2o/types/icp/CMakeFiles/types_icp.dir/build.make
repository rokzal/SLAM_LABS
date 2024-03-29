# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build

# Include any dependencies generated for this target.
include g2o/types/icp/CMakeFiles/types_icp.dir/depend.make

# Include the progress variables for this target.
include g2o/types/icp/CMakeFiles/types_icp.dir/progress.make

# Include the compile flags for this target's objects.
include g2o/types/icp/CMakeFiles/types_icp.dir/flags.make

g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.o: g2o/types/icp/CMakeFiles/types_icp.dir/flags.make
g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.o: ../g2o/types/icp/types_icp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.o"
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/types_icp.dir/types_icp.cpp.o -c /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/icp/types_icp.cpp

g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/types_icp.dir/types_icp.cpp.i"
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/icp/types_icp.cpp > CMakeFiles/types_icp.dir/types_icp.cpp.i

g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/types_icp.dir/types_icp.cpp.s"
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/icp/types_icp.cpp -o CMakeFiles/types_icp.dir/types_icp.cpp.s

# Object files for target types_icp
types_icp_OBJECTS = \
"CMakeFiles/types_icp.dir/types_icp.cpp.o"

# External object files for target types_icp
types_icp_EXTERNAL_OBJECTS =

../lib/libg2o_types_icp.so: g2o/types/icp/CMakeFiles/types_icp.dir/types_icp.cpp.o
../lib/libg2o_types_icp.so: g2o/types/icp/CMakeFiles/types_icp.dir/build.make
../lib/libg2o_types_icp.so: ../lib/libg2o_types_sba.so
../lib/libg2o_types_icp.so: ../lib/libg2o_types_slam3d.so
../lib/libg2o_types_icp.so: ../lib/libg2o_opengl_helper.so
../lib/libg2o_types_icp.so: /usr/lib/x86_64-linux-gnu/libGLU.so
../lib/libg2o_types_icp.so: /usr/lib/x86_64-linux-gnu/libGLX.so
../lib/libg2o_types_icp.so: /usr/lib/x86_64-linux-gnu/libOpenGL.so
../lib/libg2o_types_icp.so: ../lib/libg2o_core.so
../lib/libg2o_types_icp.so: ../lib/libg2o_stuff.so
../lib/libg2o_types_icp.so: g2o/types/icp/CMakeFiles/types_icp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../../../lib/libg2o_types_icp.so"
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/types_icp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
g2o/types/icp/CMakeFiles/types_icp.dir/build: ../lib/libg2o_types_icp.so

.PHONY : g2o/types/icp/CMakeFiles/types_icp.dir/build

g2o/types/icp/CMakeFiles/types_icp.dir/clean:
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp && $(CMAKE_COMMAND) -P CMakeFiles/types_icp.dir/cmake_clean.cmake
.PHONY : g2o/types/icp/CMakeFiles/types_icp.dir/clean

g2o/types/icp/CMakeFiles/types_icp.dir/depend:
	cd /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/icp /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/build/g2o/types/icp/CMakeFiles/types_icp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : g2o/types/icp/CMakeFiles/types_icp.dir/depend

