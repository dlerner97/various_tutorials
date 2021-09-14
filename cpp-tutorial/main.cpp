#include <iostream>
#include <string>
#include <iomanip>
#include <ios>
#include <vector>
#include <algorithm>
#include <stdexcept>

using std::cout;
using std::cin;
using std::endl;
using std::setprecision; // from iomanip pkg
using std::streamsize; // From ios pkg
using std::vector;
using std::sort; // From algorithm pkg
using std::domain_error;

int main(void) {
    const std::string hello{"hello"};

    // This str obj constructor creates a string of l's with size hello.size()
    std::string spaces(hello.size(), 'l');
    cout << spaces << endl;
    
    // Size_type is a data type within the string class that stores the len of chars in a string. It's basically
    // just an unsigned int but it's good practice to use their data types for instances like this
    std::string::size_type word_len{hello.size()};
    cout << "Len of hello var: " << word_len << endl;

    double x{4.56752};
    streamsize prec = cout.precision(); // Get default precision

    // Set desired precision, spit out number, then return precision to default
    cout << "Number with 2 dec places: " << setprecision(3) << x << setprecision(prec) << endl;

    // 2 or more string literals seperated by only whitespace are automatically concatenated
    cout << "Enter multiple numbers, hit end of file"
            " (try ctrl+d for linux or ctrl+z for windows) to quit: " << endl;
    double sum{};
    while (cin >> x) {
        sum += x;
    }
    cout << "Sum = " << sum << endl;

    vector<int> my_vec{4,5,6};

    // The typedef command is kinda like a macro. Creates (scoped) synonym "vec_sz" that is a direct replacement for the "vector<int>::size_type"
    typedef vector<int>::size_type vec_sz;
    vec_sz size{my_vec.size()};

    // Sorts in "nondecreasing" order in place... Basically increasing w/ duplicates
    // Can also take in a 3rd parameter. This would be a function that returns a boolean result, comparing two objects within the vector
    sort(my_vec.begin(), my_vec.end());  
    vec_sz mid = size/2;
    double median;

    // Solves median in sorted list.    
    //   If vec_size is even    return avg of 2 mid vals    else return mid val
    try{
        if(size == 0) throw domain_error("Median of empty vector!");
    } catch (domain_error e) { // catches domain error as e
        cout << e.what(); // Prints out string message of domain error ie "Median of empty vector!"
    }
    
    median = size % 2 == 0 ? (my_vec[mid] + my_vec[mid-1])/2 : my_vec[mid];

    // Never use using in header/.h files but they can be used in source/.cpp files.

    return 0;
}