# Task C.002
Author: Yixiao Wang
# What is the difference between a class and a struct in C++?
The default access mode and default inheritance mode are public if class declaration uses the struct class-key and private if the class declaration uses the class class-key.
# Why is Vector2D a struct and Transform2DClass?
Transform2D has an invariant. The data members cannot vary independently. It must follow the rules or operations defined in the class in member functions form. At the beginning, the invariant is defined by the constructor.
The access of data in Transform2D are private. But the access of data in Vector2D are public. The data in Transform2D can only be modified by the member function. But the data in Vector2D can be directly modified.
# Why are some of the constructors in Transform2D explicit?
Explicit constructors are used to avoid unnecessary implicit conversions. These functions can only be used as explicit forms. It will not convert the parameters type into the idea ones to fit the explicit constructors. Only specific parameters can activate the explicit constructors.
# Propose three different designs for normalize
1:write a function to normalize.  
2:operator overloading to use "/". When use, it will be like v/=v.  
3:turn a vector struct to class, and add a member function normalize.  

pros&cons:  
1.easy to code and the function is relatively more independent.  
2.take too much time to code and a little confusing for others to use. And others may encounter unexpected errors when operating other tasks with "/=".  
3.the data stored are private, it cannot easily assign the data or do other operations on the data from the outside.  
  
I choose the first one because it is easy for others to find the source and use. Also, it is more brief and convenient.
# Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
Transform2D::inv() does not modify the object's state but *= does.

