# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build

# Include any dependencies generated for this target.
include CMakeFiles/cozmo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cozmo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cozmo.dir/flags.make

CMakeFiles/cozmo.dir/src/cozmo.cpp.o: CMakeFiles/cozmo.dir/flags.make
CMakeFiles/cozmo.dir/src/cozmo.cpp.o: ../src/cozmo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cozmo.dir/src/cozmo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cozmo.dir/src/cozmo.cpp.o -c /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/src/cozmo.cpp

CMakeFiles/cozmo.dir/src/cozmo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cozmo.dir/src/cozmo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/src/cozmo.cpp > CMakeFiles/cozmo.dir/src/cozmo.cpp.i

CMakeFiles/cozmo.dir/src/cozmo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cozmo.dir/src/cozmo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/src/cozmo.cpp -o CMakeFiles/cozmo.dir/src/cozmo.cpp.s

CMakeFiles/cozmo.dir/src/cozmo.cpp.o.requires:

.PHONY : CMakeFiles/cozmo.dir/src/cozmo.cpp.o.requires

CMakeFiles/cozmo.dir/src/cozmo.cpp.o.provides: CMakeFiles/cozmo.dir/src/cozmo.cpp.o.requires
	$(MAKE) -f CMakeFiles/cozmo.dir/build.make CMakeFiles/cozmo.dir/src/cozmo.cpp.o.provides.build
.PHONY : CMakeFiles/cozmo.dir/src/cozmo.cpp.o.provides

CMakeFiles/cozmo.dir/src/cozmo.cpp.o.provides.build: CMakeFiles/cozmo.dir/src/cozmo.cpp.o


# Object files for target cozmo
cozmo_OBJECTS = \
"CMakeFiles/cozmo.dir/src/cozmo.cpp.o"

# External object files for target cozmo
cozmo_EXTERNAL_OBJECTS =

libcozmo.so: CMakeFiles/cozmo.dir/src/cozmo.cpp.o
libcozmo.so: CMakeFiles/cozmo.dir/build.make
libcozmo.so: /usr/lib/libdart-gui.so.6.1.1
libcozmo.so: /usr/lib/libdart-utils.so.6.1.1
libcozmo.so: /usr/lib/libdart-collision-bullet.so.6.1.1
libcozmo.so: /usr/lib/libdart.so.6.1.1
libcozmo.so: /usr/lib/x86_64-linux-gnu/libccd.so
libcozmo.so: /usr/lib/libfcl.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libassimp.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
libcozmo.so: /usr/lib/libdart-external-odelcpsolver.so.6.1.1
libcozmo.so: /usr/lib/x86_64-linux-gnu/libBulletDynamics.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libBulletCollision.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libLinearMath.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libBulletSoftBody.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libglut.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libXmu.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libXi.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libGLU.so
libcozmo.so: /usr/lib/x86_64-linux-gnu/libGL.so
libcozmo.so: /usr/lib/libdart-external-lodepng.so.6.1.1
libcozmo.so: /usr/lib/libdart-external-imgui.so.6.1.1
libcozmo.so: CMakeFiles/cozmo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libcozmo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cozmo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cozmo.dir/build: libcozmo.so

.PHONY : CMakeFiles/cozmo.dir/build

CMakeFiles/cozmo.dir/requires: CMakeFiles/cozmo.dir/src/cozmo.cpp.o.requires

.PHONY : CMakeFiles/cozmo.dir/requires

CMakeFiles/cozmo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cozmo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cozmo.dir/clean

CMakeFiles/cozmo.dir/depend:
	cd /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build /home/vinitha910/workspaces/cozmo_workspace/src/libcozmo/build/CMakeFiles/cozmo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cozmo.dir/depend
