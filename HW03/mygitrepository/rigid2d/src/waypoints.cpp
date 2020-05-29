#include<iosfwd> // contains forward definitions for iostream objects
#include<iostream>
#include<exception>
#include<math.h>
#include<vector>
#include<rigid2d/rigid2d.hpp>
#include<rigid2d/diff_drive.hpp>
#include<rigid2d/waypoints.hpp>

rigid2d::waypoints::waypoints()
{
    rigid2d::Twist2D pose;
    pose.Vx=0;
    pose.Vy=0;
    pose.Wz=0;
    diffdrive.reset(pose);
    target.x=0;
    target.y=0;
    max_Vx=1;
    max_Wz=1;
    dt=1.0e-2;
    angular_tol=1.0e-3;
    linear_tol=1.0e-3;
    num_tar=1;
    std::vector<rigid2d::Vector2D> way_p;
    rigid2d::Vector2D single_p;
    single_p.x=0;
    single_p.y=0;
    way_p.push_back(single_p);
    single_p.x=1;
    single_p.y=0;
    way_p.push_back(single_p);
    way_points=way_p;
    num_rot=0;
}
void rigid2d::waypoints::setParam(std::vector<rigid2d::Vector2D> way_p, double mVx, double mWz, double time_interval)
{
    if (way_p.size()<2)
    throw "the number of waypoints must be more than 1.";
    rigid2d::Twist2D pose;
    pose.Vx=way_p[0].x;
    pose.Vy=way_p[0].y;
    pose.Wz=0;
    diffdrive.reset(pose);
    target=way_p[1];
    max_Vx=mVx;
    max_Wz=mWz;
    dt=time_interval;
    angular_tol=1.0e-3;
    linear_tol=1.0e-3;
    num_tar=1;
    way_points=way_p;
    num_rot=0;
}


/// \brief initialize the diffdrive model.
/// \param way_p: waypoints of the target trajectory
/// \param wb: wheel_base
/// \param wr: wheel_radius
rigid2d::waypoints::waypoints(std::vector<rigid2d::Vector2D> way_p, double mVx, double mWz, double time_interval)
{
    if (way_p.size()<2)
    throw "the number of waypoints must be more than 1.";
    rigid2d::Twist2D pose;
    pose.Vx=way_p[0].x;
    pose.Vy=way_p[0].y;
    pose.Wz=0;
    diffdrive.reset(pose);
    target=way_p[1];
    max_Vx=mVx;
    max_Wz=mWz;
    dt=time_interval;
    angular_tol=1.0e-3;
    linear_tol=1.0e-3;
    num_tar=1;
    way_points=way_p;
    num_rot=0;
}
void rigid2d::waypoints::reset()
{
    rigid2d::Twist2D pose;
    pose.Vx=way_points[0].x;
    pose.Vy=way_points[0].y;
    pose.Wz=0;
    diffdrive.reset(pose);
    target=way_points[1];
    num_tar=1;
    num_rot=0;
}



/// \brief initialize the diffdrive model.
/// \param way_p: waypoints of the target trajectory
/// \param wb: wheel_base
/// \param wr: wheel_radius
rigid2d::waypoints::waypoints(std::vector<rigid2d::Vector2D> way_p, double wb, double wr, double mVx, double mWz, double time_interval)
{
    if (way_p.size()<2)
    throw "the number of waypoints must be more than 1.";
    rigid2d::Twist2D pose;
    pose.Vx=way_p[0].x;
    pose.Vy=way_p[0].y;
    pose.Wz=0;
    diffdrive.reset(pose);
    diffdrive.setParam(wb,wr);
    target=way_p[1];
    max_Vx=mVx;
    max_Wz=mWz;
    dt=time_interval;
    angular_tol=1.0e-3;
    linear_tol=1.0e-3;
    num_tar=1;
    way_points=way_p;
    num_rot=0;
}

rigid2d::Twist2D rigid2d::waypoints::nextWaypoint()
{
    rigid2d::Twist2D poseNow;
    poseNow=diffdrive.pose();
    double x,y,theta;
    theta=poseNow.Wz;
    x=target.x-poseNow.Vx;
    y=target.y-poseNow.Vy;
    if (x*x+y*y<linear_tol*linear_tol)
    {
        num_tar++;
        if (num_tar==1)
        {
            num_rot++;
        }
        if (num_tar>(int)way_points.size()-1)
        {
            num_tar=0;
        }
        target=way_points[num_tar];
        x=target.x-poseNow.Vx;
        y=target.y-poseNow.Vy;
    }

    Vector2D diffb;
    Twist2D Vb;
    diffb.x=cos(theta)*x+sin(theta)*y;
    diffb.y=-sin(theta)*x+cos(theta)*y;

    theta=atan2(diffb.y,diffb.x);

    if (fabs(theta)<angular_tol)
    {
        double Vx;
        Vx=diffb.x;
        if (Vx>max_Vx*dt)
        {
            Vx=max_Vx*dt;
        }else if (Vx<-max_Vx*dt)
        {
            Vx=-max_Vx*dt;
        }
        Vb.Wz=0;
        Vb.Vx=Vx;
        Vb.Vy=0;
    }else
    {
        double Wz;
        Wz=theta;
        if (Wz>max_Wz*dt)
        {
            Wz=max_Wz*dt;
        }else if(Wz<-max_Wz*dt)
        {
            Wz=-max_Wz*dt;
        }
        
        Vb.Wz=Wz;
        Vb.Vx=0;
        Vb.Vy=0;
    }
    diffdrive.feedforward(Vb);
    return Vb;
}

rigid2d::Twist2D rigid2d::waypoints::nextWaypointfromActualPose(rigid2d::Twist2D actual_pose)
{
    rigid2d::Twist2D poseNow;
    poseNow=actual_pose;
    double x,y,theta;
    theta=poseNow.Wz;
    x=target.x-poseNow.Vx;
    y=target.y-poseNow.Vy;
    if (x*x+y*y<linear_tol*linear_tol)
    {
        num_tar++;
        if (num_tar==1)
        {
            num_rot++;
        }
        if (num_tar>(int)way_points.size()-1)
        {
            num_tar=0;
        }
        target=way_points[num_tar];
        x=target.x-poseNow.Vx;
        y=target.y-poseNow.Vy;
    }

    Vector2D diffb;
    Twist2D Vb;
    diffb.x=cos(theta)*x+sin(theta)*y;
    diffb.y=-sin(theta)*x+cos(theta)*y;

    theta=atan2(diffb.y,diffb.x);

    if (fabs(theta)<angular_tol)
    {
        double Vx;
        Vx=diffb.x;
        if (Vx>max_Vx*dt)
        {
            Vx=max_Vx*dt;
        }else if (Vx<-max_Vx*dt)
        {
            Vx=-max_Vx*dt;
        }
        Vb.Wz=0;
        Vb.Vx=Vx;
        Vb.Vy=0;
    }else
    {
        double Wz;
        Wz=theta;
        if (Wz>max_Wz*dt)
        {
            Wz=max_Wz*dt;
        }else if(Wz<-max_Wz*dt)
        {
            Wz=-max_Wz*dt;
        }
        
        Vb.Wz=Wz;
        Vb.Vx=0;
        Vb.Vy=0;
    }
    diffdrive.feedforward(Vb);
    return Vb;
}

rigid2d::Twist2D rigid2d::waypoints::getTwistforError(Twist2D actual_ps)
{
    rigid2d::Twist2D poseNow;
    poseNow=diffdrive.pose();
    double x,y,theta;
    theta=actual_ps.Wz;
    x=poseNow.Vx-actual_ps.Vx;
    y=poseNow.Vy-actual_ps.Vy;
    Vector2D diffb;
    Twist2D Vb;
    diffb.x=cos(theta)*x+sin(theta)*y;
    diffb.y=-sin(theta)*x+cos(theta)*y;
    theta=atan2(diffb.y,diffb.x);

    Vb.Wz=theta;
    Vb.Vx=diffb.x;
    Vb.Vy=0;

    return Vb;

}

rigid2d::Vector2D rigid2d::waypoints::gettarget()
{
    return target;
}
rigid2d::Twist2D rigid2d::waypoints::getpose()
{
    return diffdrive.pose();
}
int rigid2d::waypoints::getCircle()
{
    return num_rot;
}
void rigid2d::waypoints::setCircle(int num)
{
    num_rot=num;
}
