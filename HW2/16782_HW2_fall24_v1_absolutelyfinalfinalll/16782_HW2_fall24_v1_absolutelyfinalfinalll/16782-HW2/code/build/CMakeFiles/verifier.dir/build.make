# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/nagababa/anaconda3/envs/py38/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/nagababa/anaconda3/envs/py38/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build

# Include any dependencies generated for this target.
include CMakeFiles/verifier.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/verifier.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/verifier.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/verifier.dir/flags.make

CMakeFiles/verifier.dir/src/verifier.cpp.o: CMakeFiles/verifier.dir/flags.make
CMakeFiles/verifier.dir/src/verifier.cpp.o: /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/src/verifier.cpp
CMakeFiles/verifier.dir/src/verifier.cpp.o: CMakeFiles/verifier.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/verifier.dir/src/verifier.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/verifier.dir/src/verifier.cpp.o -MF CMakeFiles/verifier.dir/src/verifier.cpp.o.d -o CMakeFiles/verifier.dir/src/verifier.cpp.o -c /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/src/verifier.cpp

CMakeFiles/verifier.dir/src/verifier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/verifier.dir/src/verifier.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/src/verifier.cpp > CMakeFiles/verifier.dir/src/verifier.cpp.i

CMakeFiles/verifier.dir/src/verifier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/verifier.dir/src/verifier.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/src/verifier.cpp -o CMakeFiles/verifier.dir/src/verifier.cpp.s

# Object files for target verifier
verifier_OBJECTS = \
"CMakeFiles/verifier.dir/src/verifier.cpp.o"

# External object files for target verifier
verifier_EXTERNAL_OBJECTS =

verifier: CMakeFiles/verifier.dir/src/verifier.cpp.o
verifier: CMakeFiles/verifier.dir/build.make
verifier: CMakeFiles/verifier.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable verifier"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/verifier.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/verifier.dir/build: verifier
.PHONY : CMakeFiles/verifier.dir/build

CMakeFiles/verifier.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/verifier.dir/cmake_clean.cmake
.PHONY : CMakeFiles/verifier.dir/clean

CMakeFiles/verifier.dir/depend:
	cd /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build /home/nagababa/Downloads/16782_HW2_fall24_v1_finalfinalll/16782-HW2/code/build/CMakeFiles/verifier.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/verifier.dir/depend

