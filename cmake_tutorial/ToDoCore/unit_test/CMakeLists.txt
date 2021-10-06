
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # force this option to ON so that Google Test will use /MD instead of /MT
    # /MD is now the default for Visual Studio, so it should be our default, too
    option(gtest_force_shared_crt
           "Use shared (DLL) run-time lib even when Google Test is built as static lib." ON)
elseif (APPLE)
    add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
endif()

add_executable(ToDoTest main.cpp ToDoTest.cpp)


# Ignores compile warnings in this gtest dir
# set_property(TARGET gtest APPEND_STRING PROPERTY COMPILE_FLAGS " -w")

target_link_libraries(ToDoTest toDoCore)

add_test(ToDoTest ToDoTest)