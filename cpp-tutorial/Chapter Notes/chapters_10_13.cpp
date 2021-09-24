/*
    Chapter 10-13
*/
#include <string>
#include <iostream>
#include <cstddef>
#include <vector>
#include <algorithm>
#include <cstring>
#include <fstream>

#include "../util_funcs.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;

/*
    Chapter 10
    Managing Memory and Low-Level Data Structures
*/
int dummy_func(int) {
    cout << "Hello! Says the func" << endl;
    return 2;
}

// DOESN'T WORK! x is a local variable and disappears before we can use the return value!
// Turns out this doesn't even compile. Yay for C++!
// int *invalid_ptr() {
//     int x{3};
//     return &x;
// }

// The following is a working replacement because "static" vars place memory on heap.
int *valid_ptr() {
    static int x{3};
    return &x;
}

// Allocates new memory loc each time. Last until the program ends or "delete p" is called. Watch for memory leaks!!!
int *better_valid_ptr() {
    return new int(3);
}

void chapter_10(int argc, char** argv) {
    // & -> Address operator
    // * -> Dereference operator
    string* str_ptr{nullptr};
    if (str_ptr == 0) cout << "Nullptr is just an int 0!" << endl;
    string* str_ptr2{0};
    // string* str_ptr2{4}; -> Does not compile! Only a 0 (i.e. nullptr) can be used.

    // Just FYI, string *p is a better notation because "string* p, q" makes a "str* p" and "str q"!

    // We can only refer to functions by their address so the following 2 lines are equivalent (besides of course the var construction)
    int (*func_ptr) (int) {&dummy_func};
    func_ptr = dummy_func;
    func_ptr(2);

    // Good practice using C++ arrays
    const size_t NDim{3};
    double coords[NDim]{4,5,6};

    cout << "Array ptrs merely point to the first element: " << *coords << endl;
    vector<double> v;
    std::copy(coords, coords + NDim, std::back_inserter(v));
    cout << "We can also use std lib techniques! " << v << endl;

    double *dbl_ptr{&coords[2]};
    ptrdiff_t ptr_diff{dbl_ptr-coords};
    cout << "Distance btwn 2 pointers in single array is length apart (i.e. 2-0): " << ptr_diff << endl;

    // If we initiaize an array immediately, we don't need to give it a size
    double coords2[]{1,2,3};

    const char hello[]{'H', 'e', 'l', 'l', 'o', '\0'};
    if (hello == "hello") cout << "A string literal is just a char array with a 'null character' ending!" << endl;

    cout << "The coords array is " << sizeof(coords)/sizeof(*coords) << " elements long" << endl;

    // argc -> Number of individual "words"
    // argv -> pointer to pointer of "words." I.e. argv points to a "list" of words, pointing to a list of individual chars (including \0's at the end of each)
    if (argc > 1) {
        cout << "Argv: ";
        for (int i = 0; i < argc; i++) {
            cout << argv[i] << " ";
        }
        cout << endl;
    } 

    // Seems like "ifstream" inherits from "istream" so we can use polymorphism here. (Same as ofstream <-> ostream)
    std::ifstream infile("in");
    // ...OR...
    string in{"in"};
    std::ifstream infile2(in.c_str());

    std::ofstream outfile("out");

    string s;
    if (infile) {
        while (getline(infile, s)) {
            outfile << s << endl;
        }
    } else cout << "File doesn't exist or cannot be read!" << endl; 

    cout << "Valid: " << *valid_ptr() << endl;
    *valid_ptr() = 4;
    cout << "However, the function will point to the same object every instance! " << *valid_ptr() << endl;

    auto p = better_valid_ptr();
    cout << "Still valid: " << *p << endl;
    *p = 4;
    cout << "The function will no longer point to the same object. THIS IS A MEMORY LEAK!!! " << *better_valid_ptr() << endl;
    
    // Brackets tell the compiler to delete the entire array not just the first element.
    // delete[] coords; 
}

/*
    Chapter 13
    Using Inheritance and Dynamic Binding
*/

class Base {
 // Private to all class and subclass users but public within the derived classes.
 protected:
    int protected_mem{3};
 public:
    // Virtual keyword tells implementation to choose the proper method at runtime. FYI, virtual funcs MUST be defined!
    virtual int virtual_func() {
        return 3;
    }

    int non_virtual_func() {
        return 3;
    }

    // Returns a pointer to an identical obj. It's a COPY, not a reference.
    virtual Base* clone() const { return new Base(*this); } 

    // Base classes should always have a destructor so that we can explicitly set it as virtual.
    // It does not need a body. We just need to insure that deleting an object frees all memory including the memory within Derived outside of Base!
    virtual ~Base() {}
};

// Inherits from Base. "Public" means that the public Base members are public within the derived class
class Derived : public Base {
 public:
    int get_protected_mem() {
        return protected_mem;
    }

    int virtual_func() {
        return 4;
    }

    int non_virtual_func() {
        return 4;
    }

    // Because this function is static, it belongs to the class not a specific obj. 
    static int get_5() {
        return 5;
    }

    virtual Derived* clone() const { return new Derived(*this); } 
};

void print_virt_func_rets(Base *obj) {
    cout << "Func return is " << obj->virtual_func() << endl;
}

void print_non_virt_func_rets(Base *obj) {
    cout << "Func return is " << obj->non_virtual_func() << endl;
}

void chapter_13() {
    Base base;
    Derived der;

    // Virtual only works if we refer to the object as a pointer!
    // Without a pointer, we are passing in the "base" portion of the "derived" class so it's identical to type "Base." Passing as
    // a pointer is more general because it merely points to the a base-class obj OR a derived type.
    print_virt_func_rets(&base);
    print_virt_func_rets(&der);
    print_non_virt_func_rets(&base);
    print_non_virt_func_rets(&der);

    cout << "Get 5: " << Derived::get_5() << endl;
}

void chapters_10_13(int argc, char** argv) {
    // chapter_10(argc, argv);
    chapter_13();
}