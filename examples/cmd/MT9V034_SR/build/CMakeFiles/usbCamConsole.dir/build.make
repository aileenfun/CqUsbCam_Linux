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
CMAKE_SOURCE_DIR = /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build

# Include any dependencies generated for this target.
include CMakeFiles/usbCamConsole.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/usbCamConsole.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/usbCamConsole.dir/flags.make

CMakeFiles/usbCamConsole.dir/main.cpp.o: CMakeFiles/usbCamConsole.dir/flags.make
CMakeFiles/usbCamConsole.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/usbCamConsole.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/usbCamConsole.dir/main.cpp.o -c /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/main.cpp

CMakeFiles/usbCamConsole.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/usbCamConsole.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/main.cpp > CMakeFiles/usbCamConsole.dir/main.cpp.i

CMakeFiles/usbCamConsole.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/usbCamConsole.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/main.cpp -o CMakeFiles/usbCamConsole.dir/main.cpp.s

CMakeFiles/usbCamConsole.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/usbCamConsole.dir/main.cpp.o.requires

CMakeFiles/usbCamConsole.dir/main.cpp.o.provides: CMakeFiles/usbCamConsole.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/usbCamConsole.dir/build.make CMakeFiles/usbCamConsole.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/usbCamConsole.dir/main.cpp.o.provides

CMakeFiles/usbCamConsole.dir/main.cpp.o.provides.build: CMakeFiles/usbCamConsole.dir/main.cpp.o


# Object files for target usbCamConsole
usbCamConsole_OBJECTS = \
"CMakeFiles/usbCamConsole.dir/main.cpp.o"

# External object files for target usbCamConsole
usbCamConsole_EXTERNAL_OBJECTS =

usbCamConsole: CMakeFiles/usbCamConsole.dir/main.cpp.o
usbCamConsole: CMakeFiles/usbCamConsole.dir/build.make
usbCamConsole: CMakeFiles/usbCamConsole.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable usbCamConsole"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usbCamConsole.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/usbCamConsole.dir/build: usbCamConsole

.PHONY : CMakeFiles/usbCamConsole.dir/build

CMakeFiles/usbCamConsole.dir/requires: CMakeFiles/usbCamConsole.dir/main.cpp.o.requires

.PHONY : CMakeFiles/usbCamConsole.dir/requires

CMakeFiles/usbCamConsole.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/usbCamConsole.dir/cmake_clean.cmake
.PHONY : CMakeFiles/usbCamConsole.dir/clean

CMakeFiles/usbCamConsole.dir/depend:
	cd /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build /home/alanliu/CqUsbCam_Linux/examples/cmd/MT9V034_SR/build/CMakeFiles/usbCamConsole.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/usbCamConsole.dir/depend

