# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/hzx/motor/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hzx/motor/build

# Include any dependencies generated for this target.
include move/CMakeFiles/moving.dir/depend.make

# Include the progress variables for this target.
include move/CMakeFiles/moving.dir/progress.make

# Include the compile flags for this target's objects.
include move/CMakeFiles/moving.dir/flags.make

move/CMakeFiles/moving.dir/src/moving.cpp.o: move/CMakeFiles/moving.dir/flags.make
move/CMakeFiles/moving.dir/src/moving.cpp.o: /home/hzx/motor/src/move/src/moving.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hzx/motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object move/CMakeFiles/moving.dir/src/moving.cpp.o"
	cd /home/hzx/motor/build/move && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/moving.dir/src/moving.cpp.o -c /home/hzx/motor/src/move/src/moving.cpp

move/CMakeFiles/moving.dir/src/moving.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/moving.dir/src/moving.cpp.i"
	cd /home/hzx/motor/build/move && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hzx/motor/src/move/src/moving.cpp > CMakeFiles/moving.dir/src/moving.cpp.i

move/CMakeFiles/moving.dir/src/moving.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/moving.dir/src/moving.cpp.s"
	cd /home/hzx/motor/build/move && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hzx/motor/src/move/src/moving.cpp -o CMakeFiles/moving.dir/src/moving.cpp.s

move/CMakeFiles/moving.dir/src/moving.cpp.o.requires:

.PHONY : move/CMakeFiles/moving.dir/src/moving.cpp.o.requires

move/CMakeFiles/moving.dir/src/moving.cpp.o.provides: move/CMakeFiles/moving.dir/src/moving.cpp.o.requires
	$(MAKE) -f move/CMakeFiles/moving.dir/build.make move/CMakeFiles/moving.dir/src/moving.cpp.o.provides.build
.PHONY : move/CMakeFiles/moving.dir/src/moving.cpp.o.provides

move/CMakeFiles/moving.dir/src/moving.cpp.o.provides.build: move/CMakeFiles/moving.dir/src/moving.cpp.o


# Object files for target moving
moving_OBJECTS = \
"CMakeFiles/moving.dir/src/moving.cpp.o"

# External object files for target moving
moving_EXTERNAL_OBJECTS =

/home/hzx/motor/devel/lib/move/moving: move/CMakeFiles/moving.dir/src/moving.cpp.o
/home/hzx/motor/devel/lib/move/moving: move/CMakeFiles/moving.dir/build.make
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/libroscpp.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/librosconsole.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/librostime.so
/home/hzx/motor/devel/lib/move/moving: /opt/ros/melodic/lib/libcpp_common.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/hzx/motor/devel/lib/move/moving: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/hzx/motor/devel/lib/move/moving: move/CMakeFiles/moving.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hzx/motor/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/hzx/motor/devel/lib/move/moving"
	cd /home/hzx/motor/build/move && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/moving.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
move/CMakeFiles/moving.dir/build: /home/hzx/motor/devel/lib/move/moving

.PHONY : move/CMakeFiles/moving.dir/build

move/CMakeFiles/moving.dir/requires: move/CMakeFiles/moving.dir/src/moving.cpp.o.requires

.PHONY : move/CMakeFiles/moving.dir/requires

move/CMakeFiles/moving.dir/clean:
	cd /home/hzx/motor/build/move && $(CMAKE_COMMAND) -P CMakeFiles/moving.dir/cmake_clean.cmake
.PHONY : move/CMakeFiles/moving.dir/clean

move/CMakeFiles/moving.dir/depend:
	cd /home/hzx/motor/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hzx/motor/src /home/hzx/motor/src/move /home/hzx/motor/build /home/hzx/motor/build/move /home/hzx/motor/build/move/CMakeFiles/moving.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : move/CMakeFiles/moving.dir/depend

