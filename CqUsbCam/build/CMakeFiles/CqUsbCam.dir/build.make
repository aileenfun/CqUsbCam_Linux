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
CMAKE_SOURCE_DIR = /home/alan/CqUsbCam_Linux/CqUsbCam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alan/CqUsbCam_Linux/CqUsbCam/build

# Include any dependencies generated for this target.
include CMakeFiles/CqUsbCam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CqUsbCam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CqUsbCam.dir/flags.make

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o: ../CqUsbCam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/CqUsbCam.cpp

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/CqUsbCam.cpp > CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.i

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/CqUsbCam.cpp -o CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.s

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.requires

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.provides: CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.provides

CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o


CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o: ../libcyusb.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/libcyusb.cpp

CMakeFiles/CqUsbCam.dir/libcyusb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/libcyusb.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/libcyusb.cpp > CMakeFiles/CqUsbCam.dir/libcyusb.cpp.i

CMakeFiles/CqUsbCam.dir/libcyusb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/libcyusb.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/libcyusb.cpp -o CMakeFiles/CqUsbCam.dir/libcyusb.cpp.s

CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.requires

CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.provides: CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.provides

CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o


CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o: ../DataProcess.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/DataProcess.cpp

CMakeFiles/CqUsbCam.dir/DataProcess.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/DataProcess.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/DataProcess.cpp > CMakeFiles/CqUsbCam.dir/DataProcess.cpp.i

CMakeFiles/CqUsbCam.dir/DataProcess.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/DataProcess.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/DataProcess.cpp -o CMakeFiles/CqUsbCam.dir/DataProcess.cpp.s

CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.requires

CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.provides: CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.provides

CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o


CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o: ../DataCapture.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/DataCapture.cpp

CMakeFiles/CqUsbCam.dir/DataCapture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/DataCapture.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/DataCapture.cpp > CMakeFiles/CqUsbCam.dir/DataCapture.cpp.i

CMakeFiles/CqUsbCam.dir/DataCapture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/DataCapture.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/DataCapture.cpp -o CMakeFiles/CqUsbCam.dir/DataCapture.cpp.s

CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.requires

CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.provides: CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.provides

CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o: ../sensors/AR0144.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0144.cpp

CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0144.cpp > CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0144.cpp -o CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o: ../sensors/SC130GS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/SC130GS.cpp

CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/SC130GS.cpp > CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/SC130GS.cpp -o CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o: ../sensors/MT9V034.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9V034.cpp

CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9V034.cpp > CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9V034.cpp -o CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o: ../sensors/MT9M001.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9M001.cpp

CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9M001.cpp > CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9M001.cpp -o CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o: ../sensors/AR0135.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0135.cpp

CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0135.cpp > CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/AR0135.cpp -o CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o


CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o: CMakeFiles/CqUsbCam.dir/flags.make
CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o: ../sensors/MT9P031.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o -c /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9P031.cpp

CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9P031.cpp > CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.i

CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alan/CqUsbCam_Linux/CqUsbCam/sensors/MT9P031.cpp -o CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.s

CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.requires:

.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.requires

CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.provides: CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.requires
	$(MAKE) -f CMakeFiles/CqUsbCam.dir/build.make CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.provides.build
.PHONY : CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.provides

CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.provides.build: CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o


# Object files for target CqUsbCam
CqUsbCam_OBJECTS = \
"CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o" \
"CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o" \
"CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o" \
"CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o" \
"CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o"

# External object files for target CqUsbCam
CqUsbCam_EXTERNAL_OBJECTS =

libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/build.make
libCqUsbCam.so: CMakeFiles/CqUsbCam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Linking CXX shared library libCqUsbCam.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CqUsbCam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CqUsbCam.dir/build: libCqUsbCam.so

.PHONY : CMakeFiles/CqUsbCam.dir/build

CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/CqUsbCam.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/libcyusb.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/DataProcess.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/DataCapture.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/AR0144.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/SC130GS.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/MT9V034.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/MT9M001.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/AR0135.cpp.o.requires
CMakeFiles/CqUsbCam.dir/requires: CMakeFiles/CqUsbCam.dir/sensors/MT9P031.cpp.o.requires

.PHONY : CMakeFiles/CqUsbCam.dir/requires

CMakeFiles/CqUsbCam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CqUsbCam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CqUsbCam.dir/clean

CMakeFiles/CqUsbCam.dir/depend:
	cd /home/alan/CqUsbCam_Linux/CqUsbCam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alan/CqUsbCam_Linux/CqUsbCam /home/alan/CqUsbCam_Linux/CqUsbCam /home/alan/CqUsbCam_Linux/CqUsbCam/build /home/alan/CqUsbCam_Linux/CqUsbCam/build /home/alan/CqUsbCam_Linux/CqUsbCam/build/CMakeFiles/CqUsbCam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CqUsbCam.dir/depend

