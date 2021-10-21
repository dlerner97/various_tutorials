#include <iostream>
    using std::cerr;
    using std::cout;
    using std::endl;

#include "ToDoCore/ToDo.hpp"

// __FILE__ and __LINE__ return the line file name and line number... Huh
#define EXPECT_EQUAL(test, expect) equalityTest( test, expect, #test, #expect, __FILE__, __LINE__ )

template < typename T1, typename T2 >
int equalityTest(const T1 testValue, const T2 expectedValue, 
                 const char* testName, const char* expectedName,
                 const char* fileName, const int lineNumber);

int main(int, char**) { // TODO: ERROR EX -> Replace this line w/ "int main(int argc, char** argv) {" then re-make
    int result{0};
    ToDo list;
    list.addTask("write code");
    list.addTask("compile");
    list.addTask("test");

    // |= -> Or equals operator... Never new that was a thing either.
    result |= EXPECT_EQUAL(list.size(), size_t(3));           // TODO: ERROR EX -> Replace this line w/ "result |= EXPECT_EQUAL(list.size(), 3); " then re-make
    result |= EXPECT_EQUAL(list.getTask(0), "write code");
    result |= EXPECT_EQUAL(list.getTask(1), "compile");
    result |= EXPECT_EQUAL(list.getTask(2), "test");

    if (result == 0) {
        cout << "Tests passed!" << endl;
    }

    return result;
}

template < typename T1, typename T2 >
int equalityTest(const T1 testValue, const T2 expectedValue, 
                 const char* testName, const char* expectedName,
                 const char* fileName, const int lineNumber) {
    if (testValue != expectedValue) {
        cerr << fileName << ":" << lineNumber << ": "
             << "Expected " << testName << " "
             << "to equal " << expectedName << " (" << expectedValue << ") "
             << "but got (" << testValue << ")" << endl; 
    }

    return (testValue != expectedValue);
}