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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/odroid/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/catkin_ws/build

# Include any dependencies generated for this target.
include beginner_tutorials/CMakeFiles/auv_camera.dir/depend.make

# Include the progress variables for this target.
include beginner_tutorials/CMakeFiles/auv_camera.dir/progress.make

# Include the compile flags for this target's objects.
include beginner_tutorials/CMakeFiles/auv_camera.dir/flags.make

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o: beginner_tutorials/CMakeFiles/auv_camera.dir/flags.make
beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o: /home/odroid/catkin_ws/src/beginner_tutorials/src/sp_auv_camera1.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/odroid/catkin_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o"
	cd /home/odroid/catkin_ws/build/beginner_tutorials && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o -c /home/odroid/catkin_ws/src/beginner_tutorials/src/sp_auv_camera1.cpp

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.i"
	cd /home/odroid/catkin_ws/build/beginner_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/odroid/catkin_ws/src/beginner_tutorials/src/sp_auv_camera1.cpp > CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.i

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.s"
	cd /home/odroid/catkin_ws/build/beginner_tutorials && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/odroid/catkin_ws/src/beginner_tutorials/src/sp_auv_camera1.cpp -o CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.s

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.requires:
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.requires

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.provides: beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.requires
	$(MAKE) -f beginner_tutorials/CMakeFiles/auv_camera.dir/build.make beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.provides.build
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.provides

beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.provides.build: beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o

# Object files for target auv_camera
auv_camera_OBJECTS = \
"CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o"

# External object files for target auv_camera
auv_camera_EXTERNAL_OBJECTS =

/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: beginner_tutorials/CMakeFiles/auv_camera.dir/build.make
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/libroscpp.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/librosconsole.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/liblog4cxx.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/librostime.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /opt/ros/indigo/lib/libcpp_common.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.8
/home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera: beginner_tutorials/CMakeFiles/auv_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera"
	cd /home/odroid/catkin_ws/build/beginner_tutorials && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/auv_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
beginner_tutorials/CMakeFiles/auv_camera.dir/build: /home/odroid/catkin_ws/devel/lib/beginner_tutorials/auv_camera
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/build

beginner_tutorials/CMakeFiles/auv_camera.dir/requires: beginner_tutorials/CMakeFiles/auv_camera.dir/src/sp_auv_camera1.cpp.o.requires
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/requires

beginner_tutorials/CMakeFiles/auv_camera.dir/clean:
	cd /home/odroid/catkin_ws/build/beginner_tutorials && $(CMAKE_COMMAND) -P CMakeFiles/auv_camera.dir/cmake_clean.cmake
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/clean

beginner_tutorials/CMakeFiles/auv_camera.dir/depend:
	cd /home/odroid/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/catkin_ws/src /home/odroid/catkin_ws/src/beginner_tutorials /home/odroid/catkin_ws/build /home/odroid/catkin_ws/build/beginner_tutorials /home/odroid/catkin_ws/build/beginner_tutorials/CMakeFiles/auv_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : beginner_tutorials/CMakeFiles/auv_camera.dir/depend

