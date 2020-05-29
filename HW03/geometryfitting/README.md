# Description
The pkg named "geometryfitting" is to establish the library geometryfitting. circlefitting function is included. 
# Listing
src:  
  geometryfitting.cpp: c++ code for geometry lib  
include:  
  include/geometryfitting.hpp: head file  
CMakeLists.txt  
package.xml  
# Instruction
Namespace: geometryfitting  

/// \brief circle fitting  
/// \param x - x coordinates   
/// \param y - y coordinates   
/// \return Eigen::VectorXf (center_x,center_y,radius,standard   deviation)   
Eigen::VectorXf circlefitting(Eigen::VectorXf x, Eigen::VectorXf y);