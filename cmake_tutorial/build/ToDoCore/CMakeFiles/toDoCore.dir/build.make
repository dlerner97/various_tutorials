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
include ToDoCore/CMakeFiles/toDoCore.dir/depend.make

# Include the progress variables for this target.
include ToDoCore/CMakeFiles/toDoCore.dir/progress.make

# Include the compile flags for this target's objects.
include ToDoCore/CMakeFiles/toDoCore.dir/flags.make

ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.o: ToDoCore/CMakeFiles/toDoCore.dir/flags.make
ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.o: ../ToDoCore/ToDo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dani/various_tutorials/cmake_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.o"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/toDoCore.dir/ToDo.cpp.o -c /home/dani/various_tutorials/cmake_tutorial/ToDoCore/ToDo.cpp

ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/toDoCore.dir/ToDo.cpp.i"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dani/various_tutorials/cmake_tutorial/ToDoCore/ToDo.cpp > CMakeFiles/toDoCore.dir/ToDo.cpp.i

ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/toDoCore.dir/ToDo.cpp.s"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dani/various_tutorials/cmake_tutorial/ToDoCore/ToDo.cpp -o CMakeFiles/toDoCore.dir/ToDo.cpp.s

# Object files for target toDoCore
toDoCore_OBJECTS = \
"CMakeFiles/toDoCore.dir/ToDo.cpp.o"

# External object files for target toDoCore
toDoCore_EXTERNAL_OBJECTS =

ToDoCore/libtoDoCore.a: ToDoCore/CMakeFiles/toDoCore.dir/ToDo.cpp.o
ToDoCore/libtoDoCore.a: ToDoCore/CMakeFiles/toDoCore.dir/build.make
ToDoCore/libtoDoCore.a: ToDoCore/CMakeFiles/toDoCore.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dani/various_tutorials/cmake_tutorial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libtoDoCore.a"
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && $(CMAKE_COMMAND) -P CMakeFiles/toDoCore.dir/cmake_clean_target.cmake
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/toDoCore.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ToDoCore/CMakeFiles/toDoCore.dir/build: ToDoCore/libtoDoCore.a

.PHONY : ToDoCore/CMakeFiles/toDoCore.dir/build

ToDoCore/CMakeFiles/toDoCore.dir/clean:
	cd /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore && $(CMAKE_COMMAND) -P CMakeFiles/toDoCore.dir/cmake_clean.cmake
.PHONY : ToDoCore/CMakeFiles/toDoCore.dir/clean

ToDoCore/CMakeFiles/toDoCore.dir/depend:
	cd /home/dani/various_tutorials/cmake_tutorial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dani/various_tutorials/cmake_tutorial /home/dani/various_tutorials/cmake_tutorial/ToDoCore /home/dani/various_tutorials/cmake_tutorial/build /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore /home/dani/various_tutorials/cmake_tutorial/build/ToDoCore/CMakeFiles/toDoCore.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ToDoCore/CMakeFiles/toDoCore.dir/depend

