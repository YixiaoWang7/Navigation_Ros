#ifndef GEOMETRYFITTING_INCLUDE_GUARD_HPP
#define GEOMETRYFITTING_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for geometry fitting. circlefitting is included.

#include<iosfwd> // contains forward definitions for iostream objects
#include<math.h>
#include <iostream>
#include <Eigen/Dense>


namespace geometryfitting
{
    /// \brief circle fitting
    /// \param x - x coordinates 
    /// \param y - y coordinates 
    /// \return Eigen::VectorXf (center_x,center_y,radius,standard deviation)
    Eigen::VectorXf circlefitting(Eigen::VectorXf x, Eigen::VectorXf y);
}
#endif