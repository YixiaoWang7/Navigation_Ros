#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for EKF in SLAM.

#include<iosfwd> 
#include<math.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <rigid2d/rigid2d.hpp>


namespace ekf
{
    class ekfDiffLaser
    {
        public:

        /// \brief createa ekf class with the initial state x is (x,y,theta)=(0,0,0)
        ekfDiffLaser();

        /// \brief createa ekf class with the initial state x is (x,y,theta)=initialx
        /// \param initialx - initial state x. should be Eigen::Vector3f
        explicit ekfDiffLaser(const Eigen::Vector3f & initialx);


        /// \brief call the ekf algorithm to next state
        /// \param observationIndex - observation tag to the corresponding observation
        /// \param observation - observations. each element is Eigen::Vector2f (r,theta)
        /// \param u - command input. it should be integrated twist : twist*timeInterval
        /// \return Eigen::Vector3f estimated pose of the robot from SLAM algorithm slamPose (x,y,theta)
        Eigen::Vector3f ekfCall(const std::vector<int> &observationIndex, const std::vector<Eigen::Vector2f> observation,const Eigen::Vector3f & u);

        /// \brief predict model or motion model. predict the x based on command input
        /// \param u - command input
        /// \return Eigen::VectorXf predited x
        Eigen::VectorXf predictx(const Eigen::Vector3f & u);

        /// \brief calculate the covariance matrix of the motion model
        /// \param u - command input
        /// \return Eigen::MatrixXf covariance matrix Q
        Eigen::MatrixXf calQ(const Eigen::Vector3f & u);


        /// \brief return the private paramters
        Eigen::MatrixXf returnR();
        Eigen::MatrixXf returnP();
        Eigen::VectorXf returnx();


        /// \brief calculate the H matrix of all the landmarks. Just for data association
        /// \param x_temp - state vector
        /// \return Eigen::MatrixXf H
        Eigen::MatrixXf returnH(const Eigen::VectorXf & x_temp);

        /// \brief calculate observation z of all the landmarks. Just for data association
        ///        note that the another input is the private paramter x.
        /// \param u - command input
        /// \return Eigen::VectorXf z_predict of all the landmarks
        Eigen::VectorXf returnz_predict(const Eigen::Vector3f & u);


        private:
        /// \brief state vector
        Eigen::VectorXf x;

        /// \brief predicted state vector based on motion model
        Eigen::VectorXf x_predict;

        /// \brief all the landmark observations
        Eigen::VectorXf z;

        /// \brief all the landmark predicted observations
        Eigen::VectorXf z_predict;

        /// \brief covariance matrix of motion model
        Eigen::MatrixXf Q;

        /// \brief covariance matrix of observation model
        Eigen::MatrixXf R;

        /// \brief covariance matrix of state vector x
        Eigen::MatrixXf P;

        /// \brief linearized form of observation model
        Eigen::MatrixXf H;

        /// \brief store the tags of landmarks in state vector
        std::vector<int> GlobalObservationIndex;

        /// \brief store the index of detected landmarks to all the landmarks in state vector
        std::vector<int> obToGlobal;

        /// \brief noiseR and noiseTheta are covariances of radius and theta
        double noiseR=1.0e-2,noiseTheta=1.0e-2;
        
    };
}

#endif