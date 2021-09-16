/*
    Chapters 7-9
*/

#include "../util_funcs.h"
#include <map>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>

using std::map;
using std::string;
using std::cout;
using std::endl;

/*
    Chapter 7
    Using Associative Containers
*/

void chapter_7() {
    // A map is a binary search tree. O(log(n)) for search, add, delete. Values stored in ascending order but are indexed like unordered maps.
    map<string, int> my_map{{"hi", 1}, {"hello", 2}};
    cout << "My map: " << my_map << endl;  // Uses template func on util_funcs.h
    cout << "Demonstrate pairs during map iteration: { ";
    for (map<string, int>::const_iterator it = my_map.begin(); it != my_map.end(); it++) {
        // When iterating through map, each value is a "pair" data struct (key is const, value is not)
        cout << it->first << ": " << it->second << ", ";
    }
    cout << "}" << endl;

    /*
        To make a map of vectors of ints, write:
        "map<int, vector<int> >" NOT "map<int, vector<int>>". That space tells the compiler that we're not using thr ">>" operator
    */

    // Be careful with the "[]" operator! This will automatically generate a key/val pair if none exist!
    cout << "My map key 'shouldn't exist': " << my_map["Shouldn't exist"] << ". Map is now: " << my_map << endl;

    // Rather use... The find method returns the last iterator if the key doesn't exist
    cout << "My map key 'doesn't exist': " << (my_map.find("doesn't exist") == my_map.end()) << ". Map is now: " << my_map << endl;

}

/*
    Chapter 8
    Writing Generic Functions
*/

// This template instantiation (I'mma call it a decorator) allows the add function to be generic
// Class is the type parameter (i.e. the type that it'll take on). During compilation/linking, the compiler will generate the specific function for use 
template<class T>
T add(T a, T b) { // As we see here "T" is a data type!
    // The "typename" keyword tells the compiler that this is the name of a type. Otherwise, it doesn't know what a vector<T> means at compile time.
    typename vector<T>::size_type vec_sz; // Not actually a useful line of code. Just used to demonstrate "typename"
    return a + b;
}

void chapter_8() {

}

void chapters_7_9() {
    chapter_7();
    chapter_8();
}