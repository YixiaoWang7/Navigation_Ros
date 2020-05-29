#ifndef WAYPOINTS_INCLUDE_GUARD_HPP
#define WAYPOINTS_INCLUDE_GUARD_HPP
/// \file
/// \brief Diffrential Drive model

#include<iosfwd> // contains forward definitions for iostream objects
#include<iostream>
#include<math.h>
#include<vector>
#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>

namespace rigid2d
{
    class waypoints
    {
        public:

        explicit waypoints(std::vector<Vector2D> way_p, double mVx, double mWz, double time_interval);


        /// \brief initialize the diffdrive model.
        /// \param way_p: waypoints of the target trajectory
        /// \param wb: wheel_base
        /// \param wr: wheel_radius
        /// \param mVx: max_Vx
        /// \param mWz: max_Wz
        /// \param time_interval
        explicit waypoints(std::vector<Vector2D> way_p, double wb, double wr, double mVx, double mWz, double time_interval);


        /// \brief set the tolerance

        /// \brief
        Twist2D nextWaypoint();

        Twist2D getpose();

        Vector2D gettarget();
        private:
        std::vector<Vector2D> way_points;
        DiffDrive diffdrive;
        Vector2D target;
        double max_Vx, max_Wz,dt;
        double angular_tol, linear_tol;
        uint num_tar;


    };
}

#endif