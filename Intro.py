myImag = 1j  # creates complex number
myBigNum = 1e3 = 1E3  # like matlab

type(var) # returns variable type

print("{0} * {1} = {2}".format(x, y, x*y)) # This will print out x*y = xy. {0} is the 0th index of the list inside format. Allows you to quickly attach variables
print("hello", "world", sep='stupid') # will print out 'hellostupidworld'. seperates the arguments by some str. default is a space

dir(__builtins__) # gives all standard python functions
help(max) # like matlab
dir(math) # gives all functions available in math modules

A = {1,5,8,3,6,8,3,6} # creates a "set." A set automatically sorts numbers in ascending order and removes repeated values.
  # --> A would then equal A = 1,3,5,6,8
  A.add(10) # adds 10 if not there
  A.update([1, 95, 9]) # adds multiple numbers

A[start:end:n] = A[::n] # slices  list b. If n=2, takes every other value

def ex(*args) # creates function. *args can take in any number of inputs. Eg. ex(4,5,43) -> args = (4,5,43) (tuple)
def ex(**args) # ^ but takes key/value pairs. Eg. ex(v1=4,v2=5,v3=43) -> args = {v1:4,v2:5,v3:43} (dictionary)

# Any class variable or method starting with 2 underscores is private and cannot be accessed outside the class

# Inheritance -> when a subclass has an "is/are" relationship with a superclass e.g. super: animal and sub: dog
#                Subclass has access to public (but not private) variables and methods of superclass.
class Subclass(Superclass): #...
    # If both sub and super class have __init__() functions, only the child will run when instance is created. To also run __init__()
    # for parent, use super().__init__() 

# Composition -> when no "is/are" relationship but when one class delegates responsibility to another.
               # Basically it's just when one class declares an instance of another class to perform some function

# Aggregation -> Like composition but instead of declaring class inside another class, a class is passed into another class as an init parameter

import builtins
help(builtins) # will give a lot of info regarding builtin modules including a list of all exceptions

try:
except Exception as e:
    print(e) # prints the error
    print(type(e)) # prints the type of error. Is good for seeing what exceptions may prop up and require handling
else: # runs if no error occurs
finally: # happens no matter what

raise Exception("message") # throws exception of type Exception with the error message "message"

class CustomException(Exception):
    def __init__(self):
        super().__init__("Custom Exception Message")