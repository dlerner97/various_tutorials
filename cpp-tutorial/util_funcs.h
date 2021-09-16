#pragma once

#include <iostream>
#include <iterator>
#include <algorithm>
#include <map>

// Not in tutorial but extremely extremely useful piece of code!
// Template allows you to use general types. Compiler will understand what type is being used for each specific usecase and will essentially
// copy in type and replace the typenames and class!
template <typename T, typename A, template <typename X, typename Y> class C>    // Declare template (kinda like a decorator)
std::ostream &operator<<(std::ostream &os, const C<T,A> &container)             // Overload output stream
{
  if(!container.empty()) {
    os << "< ";
    // This copies every container iterm into the "ostream_iterator." Basically just outputs every item in general container!
    copy(container.begin(), container.end(), std::ostream_iterator<T>(os, ", "));  
    os << ">";
  }
  return os;
}

template <class K, class V>
std::ostream& operator<<(std::ostream& os, std::map<K, V> m) {
    std::cout << "{ ";
    for (auto& val: m) {
        std::cout << val.first << ": " << val.second << ", ";
    }
    std::cout << "}";
    return os;
}