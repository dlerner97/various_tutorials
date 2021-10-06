# CMake generated Testfile for 
# Source directory: /home/dani/various_tutorials/cmake_tutorial
# Build directory: /home/dani/various_tutorials/cmake_tutorial/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(toDoTest "toDo")
set_tests_properties(toDoTest PROPERTIES  _BACKTRACE_TRIPLES "/home/dani/various_tutorials/cmake_tutorial/CMakeLists.txt;52;add_test;/home/dani/various_tutorials/cmake_tutorial/CMakeLists.txt;0;")
subdirs("ToDoCore")
