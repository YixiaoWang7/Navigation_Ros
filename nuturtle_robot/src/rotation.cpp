/// \file
/// \brief rotation node. call the service start "clockwise" or "counter-clockwise", 
///        send velocity command and let the diff model rotates for 20 circles. 
///
/// PARAMETERS:
///     /frac_vel (double): the fraction of the max velocity, which is used as velocity
///     /max_rot_v_robot (double): the max rotation velocity of the robot
/// PUBLISHES:
///     cmd_vel (geometry_msgs::Twist): velocity command sent to the robot, which is body twist
/// SERVICES:
///     start (nuturtle_robot::start): reset the odometry and let robot start 20 rotations. the arg is "clockwise" or "counter-clockwise"


#include <iosfwd> 
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/start.h>
#include <rigid2d/set_pose.h>

static ros::Publisher cmd_vel_pub;
static double w;
static geometry_msgs::Twist t;
static ros::Timer timer;
static int timer_count=-1;

void timerCallback(const ros::TimerEvent&)
{
  cmd_vel_pub.publish(t);
  timer_count++;
}


bool start_rot(nuturtle_robot::start::Request  &req,nuturtle_robot::start::Response &res)
{
  rigid2d::set_pose ps;
  ps.request.x=0;
  ps.request.y=0;
  ps.request.theta=0;
  ros::service::call("/set_pose",ps);
  ros::service::call("/fake/set_pose",ps);
  
  std::string rot;
  rot=req.rot_dir;

  double tw;
  if (rot=="clockwise")
  {
    tw=-w;
  }
  else if (rot=="counter-clockwise")
  {
    tw=w;
  }else
  {
    ROS_INFO("Please enter: clockwise or counter-clockwise");
    return true;
  }
  
  t.angular.z=tw;
  
  int n;
  double dt;
  n=ceil(200.0*rigid2d::PI/w);
  dt=2.0*rigid2d::PI/w/n;

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
      t.angular.z=0;
      cmd_vel_pub.publish(t);
      if (flag_timer>4)
      {
        break;
      }
      ros::Duration(2.0*rigid2d::PI/w/20).sleep();
      t.angular.z=tw;
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

  double max_rot_v_robot=0,frac_vel=0;

  /// get the paramter of frac_vel which means the fraction of the max rotation velocity
  n.getParam("/frac_vel",frac_vel);
  n.getParam("/max_rot_v_robot",max_rot_v_robot);


  w=double(frac_vel)*max_rot_v_robot;
  t.linear.x=0;
  t.linear.y=0;
  t.linear.z=0;
  t.angular.x=0;
  t.angular.y=0;

  cmd_vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel",100);

  ros::ServiceServer service = n.advertiseService("start", start_rot);
  ros::spin();
  return 0;
  
}