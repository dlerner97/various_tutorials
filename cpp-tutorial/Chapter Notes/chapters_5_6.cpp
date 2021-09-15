/*
    Chapters 5-6
*/

#include <iostream>
#include <vector>
#include <list>     // Container type. Does not support indexing but provides efficient additions/removals to middle of container
#include <cctype>   // Contains useful functions for processing individual chars
#include <algorithm> 
#include <string>
#include <numeric>  // Contains various numeric calculations

using std::vector;
using std::cout;
using std::endl;
using std::ostream;
using std::list;                // <list>
using std::isspace;             // <cctype>
using std::string;              // <string>
using std::copy;                // <algorithm>
using std::back_inserter;       // <algorithm>
using std::find_if;             // <algorithm>
using std::equal;               // <algorithm>
using std::isalnum;             // <cctype>
using std::find;                // <algorithm>
using std::search;              // <algorithm>
using std::transform;           // <algorithm>
using std::accumulate;          // <numeric>
using std::remove_copy;         // <algorithm>
using std::stable_partition;    // <algorithm>

ostream& operator<<(ostream& os, const vector<int>& vec) {
    os << "<";
    for(const auto& val: vec) {
        os << val << ", ";
    }
    os << ">";
    return os;
}

ostream& operator<<(ostream& os, const list<int>& ls) {
    os << "<";
    for(const auto& val: ls) {
        os << val << ", ";
    }
    os << ">";
    return os;
}

ostream& operator<<(ostream& os, const vector<string>& vec) {
    os << "<";
    for(const auto& val: vec) {
        os << val << ", ";
    }
    os << ">";
    return os;
}

ostream& operator<<(ostream& os, const vector<bool>& vec) {
    os << "<";
    for(const auto& val: vec) {
        os << val << ", ";
    }
    os << ">";
    return os;
}

/*
    Chapter 5
    Using Sequential Containers and Analyzing Strings
*/

void chapter_5(void) {
    // Erasing from a vector. Note that this is an O(n^2) process i.e. not good.
    vector<int> vals{1,2,3};
    cout << "Before erase: " << vals;
    vals.erase(vals.begin()+1);
    cout << ". After erase: " << vals << endl;

    // The following for loop uses lib defined iterators to access each element in vector. Iterators have significantly less overhead.
    // Note the for loop uses a "const_iterator" because we only need read-access memory. "vector<int>::iterator" can be used for read/write access.
    cout << "Iterating through vector... ";
    for (vector<int>::const_iterator iter{vals.begin()}; iter != vals.end(); iter++) { // vec.begin() and vec.end() both return iterators that can be
        cout << *iter << " ";                                                          // converted to const_iterators.
        // Dereferencing an iterator gives us the value it corresponds to! So apparently, an iterator is just a pointer to a given element. 

        // We can also directly delete an iterable w/...
        iter = vals.erase(iter) - 1; 
        // "iter" must be redefined because we erase elements from the vector. Because we increment iter each iteration of the for loop and because
        // vals.end() is called in each iteration, we must decrement iter (essentially keep iter the same) or else the condition will always be 
        // true after the first loop.
        
    }
    cout << endl << "After iter erase: " << vals << endl;

    // Note that the std library uses the same method names for shared containers. This means we can write a whole program w/ vectors and instantly
    // switch just the type to lists if necessary without changing much else. Note "[]" doesn't work with lists! I think this is really a linked list.
    list<int> ls{1,2,3};
    cout << "List: another iterable container...: " << ls << endl;

    cout << "\\n is whitespace: " << (isspace('\n') != 0) << ". n is whitespace: " << (isspace('n') != 0) << endl;
    string hello{"Hello!"};
    // string.substr(first char, number of chars)
    cout << "Substring of hello letters 2-4 (indexed at 0): " << hello.substr(2, 3) << endl;

    // Vector concatenation
    typedef vector<int> i_vec;
    i_vec vec1{1,2,3};
    i_vec vec2{4,5,6};
    vec1.insert(vec1.end(), vec2.begin(), vec2.end());
    cout << "Vec 1 concatenated w/ vec 2: " << vec1 << endl;

}

/*
    Chapter 6
    Using Library Algorithms
*/

// This function is needed because isspace is overloaded. Since the func will be passed in as a parameter, the lack of func
// arguments will confuse the compiler and will not allow proper usage.
bool is_space(char c) {
    return isspace(c);
}

bool is_palindrome(const string s) {
    return equal(s.begin(), s.end(), s.rbegin()); // s.rbegin is an iterator starting from reverse order
}

vector<int> sample_func(void) {
    return vector<int>{1,2,3};
}

// This function displays syntax for passing function in as argument.
void print_function_results(vector<int> func(void)) {
    auto vec = func();
    cout << "Print function result: ";
    for (auto val: vec) cout << val << ", ";
    cout << endl;
}

void chapter_6(void) {

    // A more general way to concatenate 2 containers. Notice the function call is exactly the same for both types of containers!
    typedef vector<int> i_vec;
    i_vec vec1 = {1,2,3};
    i_vec vec2 = {4,5,6};
    
    // Copy is what's called a "generic algorithm" -> not particular to specific container. 
    // back_inserter ""        "iterator adapter"  -> allows generic algorithm to adapt to container type.
    copy(vec2.begin(), vec2.end(), back_inserter(vec1)); // copy(begin, end, out)

    list<int> ls1{1,2,3};
    list<int> ls2{4,5,6};
    copy(ls2.begin(), ls2.end(), back_inserter(ls1));
    cout << "Vector concat: " << vec1 << ". List concat: " << ls1 << endl;

    // find_if algorithm
    string str{"Hello, I'm Dani!"};
    // find_if finds the iterator within a container sequence that follows the boolean passed-in function. 
    // In this case, "is_space" looks for a white-space char.
    cout << "First letter after white space is: " << *(++find_if(str.begin(), str.end(), is_space)) << endl;

    // Other useful <algorithm> algorithms
    cout << "Hello is palindrome: " << is_palindrome("Hello") << ". level is palindrome: " << is_palindrome("level") << endl;

    // a "static" variable is preserved across all calls for their given function scope. So if a variable is declared static, then
    // it only ever gets created on the first ever function call.

    // isalnum checks if char is a letter or digit
    cout << "5 is alphanumeric value: " << (isalnum('5') != 0) << ". \% is alphanumeric value: " << (isalnum('%') != 0) << endl;

    // find function
    static const string nonalnum = "~;/?:@=&$-_.+!*'(),";
    cout << "Found 1: " << (find(nonalnum.begin(), nonalnum.end(), 'r') != nonalnum.end()) << ". Found @: " << (find(nonalnum.begin(), nonalnum.end(), '@') !=  nonalnum.end()) << endl;

    // Search function
    string desired{"is"};
    string searching1{"Dani is the coolest!"};
    string searching2{"Dani the coolest!"};
    cout << "Found 'is' in 1st search: " << (search(searching1.begin(), searching1.end(), desired.begin(), desired.end()) != searching1.end());
    cout << ". Found 'is' in 2nd search: " << (search(searching2.begin(), searching2.end(), desired.begin(), desired.end()) != searching2.end()) << endl;

    // Iterators support indexing compared to current iterable!
    i_vec my_vec{1,2,3};
    i_vec::iterator my_iter{my_vec.begin()+1};
    cout << "Current iterator value: " << *my_iter << ". Prev iterator value: " << my_iter[-1] << endl;

    // Transform function
    vector<string> words{"level", "hello", "hi", "racecar"};
    vector<bool> palindromes_bool{};
    transform(words.begin(), words.end(), back_inserter(palindromes_bool), is_palindrome);
    cout << "List of words: " << words << ". And whether they're palindromes: " << palindromes_bool << ", respectively." << endl;

    // Passing function as argument
    print_function_results(sample_func);

    // Accumulate function -> accumulate(begin, end, initial value). Make sure the last value contains the same type as your output!!!
    cout << "Sum of: " << my_vec << " = " << accumulate(my_vec.begin(), my_vec.end(), 0) << endl;

    // Remove copy function. remove_copy(begin, end, loc, removed value). (Same as remove except copies into new var rather than in place).
    i_vec non_one{};
    remove_copy(my_vec.begin(), my_vec.end(), back_inserter(non_one), 1);
    cout << my_vec << " without the ones: " << non_one << endl;
    // remove_copy_if is similar but takes a function w/ bool return rather than a value.

    // stable_partition function. partition function is similar but doesn't necessarily retain the order within the predicates func.
    // This function orders a container by the predicate's return then returns the iterator that splits the two sections.
    cout << "Un-ordered words: " << words << ". ";
    auto first_false = stable_partition(words.begin(), words.end(), is_palindrome);
    cout << "Ordered words by palindrome: " << words << ". First false value: " << *first_false << endl;

    /* 
        IMPORTANT POINT
        Algorithms act on container elements—they do not act on containers!
        remove_if does not return a shortened container but merely swaps the order. This is necessary to be as general as it is.
        To actually erase values we need to say
   
            my_vector.erase(remove_if(my_vector.begin(), my_vector.end(), some_predicate_func), my_vector.end())

        This way, the vector actually erases all elements we'd like to remove. It does seem, however, that the remove_if func changes the definition
        of container.end().
    */
}

void chapters_5_6(void) {
    chapter_5();
    chapter_6();
}

