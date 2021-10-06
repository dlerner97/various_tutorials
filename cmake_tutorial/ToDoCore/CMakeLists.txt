# This CMake is needed to specify that this dir contains a library. It'll now be called toDoCore with the exe ToDo.cpp
# Can also state whether the lib is a static, shared, or module lib.
# STATIC -> archives of obj files that are linked directly into other targets (DEFAULT unless BUILD_SHARED_LIBS var says otherwise)
# SHARED -> linked dynamically and loaded at runtime 
# MODULE -> Plug-ins that aren't linked against but can be loaded dynamically at runtime
add_library(toDoCore ToDo.cpp)

add_subdirectory(unit_test)