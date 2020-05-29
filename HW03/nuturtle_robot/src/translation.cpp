/// \file
/// \brief translation node. call the service start "forward" or "backward", 
///        send velocity command and let the diff model move 2 meters. 
///
/// PARAMETERS:
///     /frac_vel (double): the fraction of the max velocity, which is used as velocity
///     //max_trans_v (double): the max translation velocity of the robot
/// PUBLISHES:
///     cmd_vel (geometry_msgs::Twist): velocity command sent to the robot, which is body twist
/// SERVICES:
///     start (nuturtle_robot::start): reset the odometry and let robot start to move 2 meters. the arg is "forward" or "backward"


#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/start.h>
#include <rigid2d/set_pose.h>

static ros::Publisher cmd_vel_pub;
static double Vx;
static geometry_msgs::Twist t;
static ros::Timer timer;
static int timer_count=-1;

void timerCallback(const ros::TimerEvent&)
{
  cmd_vel_pub.publish(t);
  timer_count++;
}


bool start_trans(nuturtle_robot::start::Request  &req,nuturtle_robot::start::Response &res)
{

  rigid2d::set_pose ps;
  ps.request.x=0;
  ps.request.y=0;
  ps.request.theta=0;
  ros::service::call("/set_pose",ps);
  ros::service::call("/fake/set_pose",ps);
  std::string rot;
  rot=req.rot_dir;

  double tVx;
  if (rot=="forward")
  {
    tVx=Vx;
  }
  else if (rot=="backward")
  {
    tVx=-Vx;
  }else
  {
    ROS_INFO("Please enter: forward or backward");
    return true;
  }
  
  t.linear.x=tVx;
  
  int n;
  double dt;
  n=ceil(20.0/Vx);
  dt=0.2/double(n)/Vx;

  ros::NodeHandle n1;
  timer = n1.createTimer(ros::Duration(dt), timerCallback);
  static int flag_timer=1;

  while (ros::ok())
  {
    ros::spinOnce();
    if (timer_count==n)
    {
      timer.stop();
      flag_timer++;
      timer_count=-1;
      t.linear.x=0;
      cmd_vel_pub.publish(t);
      if (flag_timer>10)
      {
        break;
      }
      ros::Duration(0.2/Vx/20).sleep();
      t.linear.x=tVx;
      timer.start();
    }
  }
  flag_timer=1;
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rotation");

  ros::NodeHandle n;

  double max_trans_v=0,frac_vel=0;
  /// get the paramter of frac_vel which means the fraction of the max rotation velocity
  n.getParam("/frac_vel",frac_vel);
  n.getParam("/max_trans_v",max_trans_v);
  Vx=double(frac_vel)*max_trans_v;

  t.linear.x=0;
  t.linear.y=0;
  t.linear.z=0;
  t.angular.x=0;
  t.angular.y=0;
  t.angular.z=0;

  rigid2d::set_pose ps;
  ps.request.x=0;
  ps.request.y=0;
  ps.request.theta=0;
  ros::service::call("/set_pose",ps);

  cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",100);

  /// \brief add service set_pose 
  /// \param pose: Twist2D
  ros::ServiceServer service = n.advertiseService("start", start_trans);
  ros::spin();
  return 0;
  
}