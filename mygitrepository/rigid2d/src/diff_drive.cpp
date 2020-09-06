#include "rigid2d/diff_drive.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include<iostream>
#include<rigid2d/rigid2d.hpp>
#include<exception>

rigid2d::DiffDrive::DiffDrive()
{
    poseNow.Wz=0;
    poseNow.Vx=0;
    poseNow.Vy=0;
    wheel_base=1.0;
    wheel_radius=0.2;
    angle_left=0;
    angle_right=0;
    angle_left_encoder=0;
    angle_right_encoder=0;
    WV_encoder.w_left=0;
    WV_encoder.w_right=0;
}

rigid2d::DiffDrive::DiffDrive(const Twist2D & p, double wb, double wr)
{
    poseNow.Wz=p.Wz;
    poseNow.Vx=p.Vx;
    poseNow.Vy=p.Vy;
    wheel_base=wb;
    wheel_radius=wr;    
    angle_left=0;
    angle_right=0;
    angle_left_encoder=0;
    angle_right_encoder=0;
    WV_encoder.w_left=0;
    WV_encoder.w_right=0;
}

void rigid2d::DiffDrive::setParam(double wb, double wr)
{
    wheel_base=wb;
    wheel_radius=wr;
}

rigid2d::WheelVelocities rigid2d::DiffDrive::twistToWheels(const rigid2d::Twist2D & t)
{
    if (fabs(t.Vy)>1.0e-8)
    throw "The Vy of the body twist in differential drive model must be zeros.";
    if (wheel_radius<1.0e-8)
    throw "wheel_radius is too small";
    rigid2d::WheelVelocities wv;
    wv.w_left=(-wheel_base/2*t.Wz+t.Vx)/wheel_radius;
    wv.w_right=(wheel_base/2*t.Wz+t.Vx)/wheel_radius;
    return wv;
}

rigid2d::Twist2D rigid2d::DiffDrive::wheelsToTwist(const rigid2d::WheelVelocities wv)
{
    rigid2d::Twist2D t;
    t.Vy=0;
    t.Vx=(wv.w_left+wv.w_right)/2*wheel_radius;
    t.Wz=(wv.w_right-wv.w_left)*wheel_radius/wheel_base;
    return t;
}

void rigid2d::DiffDrive::updateOdometry(double left, double right)
{
    WV_encoder.w_left=left-angle_left_encoder;
    WV_encoder.w_right=right-angle_right_encoder;
    feedforward(wheelsToTwist(WV_encoder));
    angle_left_encoder=left;
    angle_right_encoder=right;
}

void rigid2d::DiffDrive::feedforward(const rigid2d::Twist2D & cmd)
{
    if (fabs(cmd.Wz)<1.0e-8)
    {
        rigid2d::Twist2D dq;
        dq.Wz=0;
        dq.Vx=cos(poseNow.Wz)*cmd.Vx-sin(poseNow.Wz)*cmd.Vy;
        dq.Vy=sin(poseNow.Wz)*cmd.Vx+cos(poseNow.Wz)*cmd.Vy;
        poseNow.Vx+=dq.Vx;
        poseNow.Vy+=dq.Vy;
        poseNow.Wz+=dq.Wz;
    }else
    {
        rigid2d::Twist2D dq, dqb;
        dqb.Wz=cmd.Wz;
        dqb.Vx=(sin(cmd.Wz)*cmd.Vx+cmd.Vy*(cos(cmd.Wz)-1))/cmd.Wz;
        dqb.Vy=(sin(cmd.Wz)*cmd.Vy+cmd.Vx*(-cos(cmd.Wz)+1))/cmd.Wz;
        dq.Wz=dqb.Wz;
        dq.Vx=cos(poseNow.Wz)*dqb.Vx-sin(poseNow.Wz)*dqb.Vy;
        dq.Vy=sin(poseNow.Wz)*dqb.Vx+cos(poseNow.Wz)*dqb.Vy;
        poseNow.Vx+=dq.Vx;
        poseNow.Vy+=dq.Vy;
        poseNow.Wz+=dq.Wz;
    }
    poseNow.Wz=normalize_angle(poseNow.Wz);
    
    rigid2d::WheelVelocities wv;
    wv=twistToWheels(cmd);
    angle_left+=wv.w_left;
    angle_right+=wv.w_right;
}

rigid2d::Twist2D rigid2d::DiffDrive::pose()
{
    return poseNow;
}

rigid2d::WheelVelocities rigid2d::DiffDrive::wheelVelocities() const
{
    return WV_encoder;
}

void rigid2d::DiffDrive::reset(rigid2d::Twist2D ps)
{
    poseNow.Wz=ps.Wz;
    poseNow.Vx=ps.Vx;
    poseNow.Vy=ps.Vy;   
    angle_left=0;
    angle_right=0;
    angle_left_encoder=0;
    angle_right_encoder=0;
    WV_encoder.w_left=0;
    WV_encoder.w_right=0;
}

void rigid2d::DiffDrive::set_pose(rigid2d::Twist2D ps)
{
    poseNow.Wz=ps.Wz;
    poseNow.Vx=ps.Vx;
    poseNow.Vy=ps.Vy;   
    angle_left_encoder=0;
    angle_right_encoder=0;
    angle_left=0;
    angle_right=0;
    
}

/// \brief get the wheel(/joint) state with feedforward method
rigid2d::WheelVelocities rigid2d::DiffDrive::getJointStateFeedforward() const
{
    WheelVelocities jointstates;
    jointstates.w_left=angle_left;
    jointstates.w_right=angle_right;
    return jointstates;
}