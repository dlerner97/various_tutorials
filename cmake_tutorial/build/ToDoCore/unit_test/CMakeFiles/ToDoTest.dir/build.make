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
CMAKE_SOURCE_DIR = /home/dani/various_tutorials/cmake_tutorial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dani/various_tutorials/cmake_tutorial/build

# Include any dependencies generated for this target.
include ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/depend.make

# Include the progress variables for this target.
include ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/progress.make

# Include the compile flags for this target's objects.
include ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/flags.make

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.o: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/flags.make
ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.o: ../ToDoCore/unit_test/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dani/various_tutorials/cmake_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.o"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ToDoTest.dir/main.cpp.o -c /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/main.cpp

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ToDoTest.dir/main.cpp.i"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/main.cpp > CMakeFiles/ToDoTest.dir/main.cpp.i

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ToDoTest.dir/main.cpp.s"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/main.cpp -o CMakeFiles/ToDoTest.dir/main.cpp.s

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/flags.make
ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o: ../ToDoCore/unit_test/ToDoTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dani/various_tutorials/cmake_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o -c /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/ToDoTest.cpp

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ToDoTest.dir/ToDoTest.cpp.i"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/ToDoTest.cpp > CMakeFiles/ToDoTest.dir/ToDoTest.cpp.i

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ToDoTest.dir/ToDoTest.cpp.s"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test/ToDoTest.cpp -o CMakeFiles/ToDoTest.dir/ToDoTest.cpp.s

# Object files for target ToDoTest
ToDoTest_OBJECTS = \
"CMakeFiles/ToDoTest.dir/main.cpp.o" \
"CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o"

# External object files for target ToDoTest
ToDoTest_EXTERNAL_OBJECTS =

ToDoCore/unit_test/ToDoTest: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/main.cpp.o
ToDoCore/unit_test/ToDoTest: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/ToDoTest.cpp.o
ToDoCore/unit_test/ToDoTest: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/build.make
ToDoCore/unit_test/ToDoTest: ToDoCore/libtoDoCore.a
ToDoCore/unit_test/ToDoTest: ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dani/various_tutorials/cmake_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ToDoTest"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ToDoTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/build: ToDoCore/unit_test/ToDoTest

.PHONY : ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/build

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/clean:
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test && $(CMAKE_COMMAND) -P CMakeFiles/ToDoTest.dir/cmake_clean.cmake
.PHONY : ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/clean

ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/depend:
	cd /home/dani/various_tutorials/cmake_tutorial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dani/various_tutorials/cmake_tutorial /home/dani/various_tutorials/cmake_tutorial/ToDoCore/unit_test /home/dani/various_tutorials/cmake_tutorial/build /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ToDoCore/unit_test/CMakeFiles/ToDoTest.dir/depend

