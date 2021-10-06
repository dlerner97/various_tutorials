# While the following command is optional, it is recommended in all systems to ensure robustness.
# Basically disables the build unless an assigned minimum version exists on the computer.
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Removes warning on cygwin systems. Has no effect on other OS's.
set(CMAKE_LEGACY_CYGWIN_WIN32 0)

# CMakelists file must always have the "project(name)" statement. It corresponds to the project name.
# Optionally, one can also specify the language (CXX, C, Java, Fortran) but CMake defaults to C or CXX
project("To Do List")

# Enables the add_test() command. Should only be used in top-level CMake.
enable_testing()

# The following turns on warnings and treats them as compilation errors.
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(warnings "/W4 /WX /EHsc")
endif()

# These two lines would theoretically enable the warnings every time "make" is called. Rather, we move this into the following if statement.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${warnings}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${warnings}")

# As I understand the following section, it sets the CMAKE_CXX_FLAGS and CMAKE_C_FLAGS upon the FIRST build. These flags are stored
# in a cache and don't need to be redefined every time. Therefore, we check if we've previously built, and if not only then do we set them. 
if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${warnings}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS "${warnings}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
endif()

# Adds the args to the end of this dir's include path. Only needed when subdir's exist. CMAKE_CURRENT_SOURCE_DIR contains the full path to the current CMake dir
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Includes the directory in the project. This dir MUST contain its own CMakeLists.txt
add_subdirectory(ToDoCore)

# The "add_exectuable(target sources)" statement tells CMake you want to make an exe and adds it as target. It also is required for every CMakeLists.txt
# First arg is the target exe name while the 2nd-nth are the source files it'll need to compile. (Header files should NOT be listed!)
# Note: the first "add_exe...()" statement was made for chapters 1-3. In chapter 4, we reconstruct ToDo as a library so we will need a new add_exe statement
# add_executable(toDo main.cpp ToDo.cpp)
add_executable(toDo main.cpp)

# Specify that "toDo" must be linked to a library.
target_link_libraries(toDo toDoCore)

# Adds a testing script to the current dir that will be run by CTest. 
add_test(toDoTest toDo)

# Sets cache flag that tells CMake that it's already been compiled.
set(CONFIGURED_ONCE TRUE CACHE INTERNAL 
    "A flag showing that CMake has configured at least once.")