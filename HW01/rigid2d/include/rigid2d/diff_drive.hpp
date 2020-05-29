#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Diffrential Drive model

#include<iosfwd> // contains forward definitions for iostream objects
#include<iostream>
#include<math.h>
#include<rigid2d/rigid2d.hpp>


namespace rigid2d
{
    struct WheelVelocities
    {
        double w_left=0.0;
        double w_right=0.0;
    };
    class DiffDrive
    {
        public:
        /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
        DiffDrive();

        /// \brief create a DiffDrive model by specifying the pose, and geometry
        ///
        /// \param p - the current position of the robot
        /// \param wb - the distance between the wheel centers
        /// \param wr - the raidus of the wheels
        explicit DiffDrive(const Twist2D & p, double wb, double wr);

        /// \brief set the wheel_base and wheel_radius parameter
        void setParam(double wb, double wr);

        /// \brief determine the wheel velocities required to make the robot
        ///        move with the desired linear and angular velocities
        /// \param t - the desired twist in the body frame of the robot
        /// \returns - the wheel velocities to use
        /// \throws if wheel_radius is too small
        /// \see part 1) in /doc/Kinematics.pdf
        WheelVelocities twistToWheels(const Twist2D & t);

        /// \brief determine the body twist of the robot from its wheel velocities
        /// \param wv - the velocities of the wheels, assumed to be held constant
        ///  for one time unit
        /// \returns twist in the original body frame of the model
        /// \see part 2) in /doc/Kinematics.pdf
        Twist2D wheelsToTwist(const WheelVelocities wv);

        /// \brief Update the robot's odometry based on the current encoder readings
        /// \param left - the left encoder angle (in radians)
        /// \param right - the right encoder angle (in radians)
        /// \return the velocities of each wheel, assuming that they have been
        /// constant since the last call to updateOdometry
        void updateOdometry(double left, double right);

        /// \brief update the odometry of the diff drive robot, assuming that
        /// it follows the given body twist for one time  unit
        /// \param cmd - the twist command to send to the robot
        /// \see part 3) in /doc/Kinematics.pdf
        void feedforward(const Twist2D & cmd);

        /// \brief get the current pose of the robot
        Twist2D pose();

        /// \brief get the wheel speeds, based on the last encoder update
        WheelVelocities wheelVelocities() const;

        /// \brief reset the robot to the given position/orientation
        void reset(Twist2D ps);
        
        /// \brief get the wheel(/joint) state with feedforward method
        WheelVelocities getJointStateFeedforward() const;

        private:
        Twist2D poseNow;
        double wheel_base, wheel_radius;
        double angle_right, angle_left, angle_right_encoder, angle_left_encoder;
        WheelVelocities WV_encoder;
    };
}

#endif
