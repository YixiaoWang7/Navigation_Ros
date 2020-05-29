/// \file
/// \brief feedward and feedback control node. input the waypoints (x,y), subscribe the odom to get the 
///        actual position, use feedward and feedback control to let the turtlebot follow the expected 
///        trajectory.
///
/// PARAMETERS:
///     waypoint_x (str): the x coordinates of way points which are spaced by one blank space
///     waypoint_y (str): the y coordinates of way points which are spaced by one blank space
///     wheel_radius (double): wheel radius of the diff model
///     wheel_base (double): the distance between the centers of two wheels
///     max_trans_v (double): the max translation velocity of the robot
///     max_rot_v_robot (double): the max rotation velocity of the robot
///     /odometer/left_wheel_joint (str): the name of left wheel joint
///     /odometer/right_wheel_joint (str): the name of right wheel joint
///     kp_l (double): kp paramter for the pid controller of translation
///     kd_l (double): kd paramter for the pid controller of translation
///     ki_l (double): ki paramter for the pid controller of translation
///     kp_w (double): kp paramter for the pid controller of rotation
///     kd_w (double): kd paramter for the pid controller of rotation
///     ki_w (double): ki paramter for the pid controller of rotation
/// PUBLISHES:
///     cmd_vel (geometry_msgs::Twist): velocity command sent to the robot, which is body twist
///     visualization_marker (visualization_msgs::Marker): marker the waypoints in the rviz
/// SUBSCRIBES:
///     odom (nav_msgs::Odometry): odometry based on the joint states from turtlebot
/// SERVICES:
///     start (std_srvs::Empty): start the trajectory following the way points
///     stop (std_srvs::Empty): stop the trajectory following the way points

#include "ros/ros.h"
#include <vector>
#include <math.h>
#include "std_msgs/String.h"
#include <turtlesim/Pose.h>
#include "geometry_msgs/Twist.h"
#include <string>
#include <tsim/PoseError.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <std_srvs/Empty.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/waypoints.hpp>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rigid2d/set_pose.h>
#include <nav_msgs/Odometry.h>

static double max_trans_v=0,max_rot_v_robot=0,frac_vel=0;
static bool flag_way=0;
static std::string left_wheel_joint, right_wheel_joint;
static rigid2d::set_pose ps;
static rigid2d::Twist2D initial_pose;
static bool resetCircle=0;
static rigid2d::Twist2D actual_pose;
static ros::Publisher marker_pub;
static std::vector<rigid2d::Vector2D> way_p;
static visualization_msgs::Marker marker;
static double kp_l=0,ki_l=0,kd_l=0,kp_w=0,ki_w=0,kd_w=0;

void odomBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  actual_pose.Vx=msg->pose.pose.position.x;
  actual_pose.Vy=msg->pose.pose.position.y;
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  actual_pose.Wz=yaw;
}

std::vector<double> char2vec(char*x)
{
  int num=strlen(x);
  std::vector<int> space_in;
  space_in.push_back(-1);
  for (int i=0;i<num;i++)
  {
    if (x[i]==' ')
    {
      space_in.push_back(i);
    }
  }
  space_in.push_back(num);

  std::vector<double> an;

  for (int i=0;i<(int)space_in.size()-1;i++)
  {
    double t_num=0;
    int point_in=-1;
    for (int j=space_in[i]+1;j<space_in[i+1];j++)
    {
      if (x[j]=='.')
      {
        point_in=j;
        break;
      }
    }
    if (point_in==-1)
    {
      for (int j=space_in[i]+1;j<space_in[i+1];j++)
      {
        t_num=t_num*10+(int)(x[j]-'0');
      }
    }else
    {
      double t_point=0;
      for (int j=space_in[i]+1;j<point_in;j++)
      {
        t_num=t_num*10+(int)(x[j]-'0');
      } 
      for (int j=space_in[i+1]-1;j>point_in;j--)
      {
        t_point=t_point*0.1+(int)(x[j]-'0');
      } 
      t_num+=t_point*0.1;
    }
    an.push_back(t_num);
  }
  return an;
}

bool start_way(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
  ros::NodeHandle nstart;
  nstart.getParam("kp_l",kp_l);
  nstart.getParam("ki_l",ki_l);
  nstart.getParam("kd_l",kd_l);
  nstart.getParam("kp_w",kp_w);
  nstart.getParam("ki_w",ki_w);
  nstart.getParam("kd_w",kd_w);
    for (int i=0;i<(int)way_p.size();i++)
  {
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x=way_p[i].x;
    marker.pose.position.y=way_p[i].y;
    marker.pose.position.z=0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    marker.pose.orientation.x=q.x();
    marker.pose.orientation.y=q.y();
    marker.pose.orientation.z=q.z();
    marker.pose.orientation.w=q.w();
    marker_pub.publish(marker);
    marker.id+=1;
  }
  flag_way=1;
  ros::service::call("/set_pose",ps);
  ros::service::call("/fake/set_pose",ps);
  resetCircle=1;
  return true;
}

bool stop_way(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
  flag_way=0;
  return true;
}

int main(int argc, char **argv)
{
  //define the node called turtle_way
  ros::init(argc, argv, "real_waypoint");
  ros::NodeHandle n;
  
  std::string waypoint_x,waypoint_y;
  n.getParam("waypoint_x",waypoint_x);
  n.getParam("waypoint_y",waypoint_y);
  n.getParam("/odometer/right_wheel_joint",right_wheel_joint);
  n.getParam("/odometer/left_wheel_joint",left_wheel_joint);
  std::vector<double> x=char2vec(waypoint_x.data());
  std::vector<double> y=char2vec(waypoint_y.data());
  for (int i=0;i<(int)x.size();i++)
  {
    rigid2d::Vector2D sp;
    sp.x=x[i];
    sp.y=y[i];
    way_p.push_back(sp);
    //ROS_INFO("%f %f",sp.x,sp.y);
  }
  ps.request.x=way_p[0].x;
  ps.request.y=way_p[0].y;
  ps.request.theta=0;

  

  double wb=10,wr=10;
  n.getParam("wheel_radius",wr);
  n.getParam("wheel_base",wb);
  n.getParam("max_trans_v",max_trans_v);
  n.getParam("max_rot_v_robot",max_rot_v_robot);
  
  n.getParam("/frac_vel",frac_vel);
  ros::ServiceServer start = n.advertiseService("start", start_way);
  ros::ServiceServer stop = n.advertiseService("stop", stop_way);

  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);

  double max_Vx=0,max_Wz=0;
  max_Vx=max_trans_v*frac_vel;
  max_Wz=max_rot_v_robot*frac_vel;

  geometry_msgs::Twist cmd;
  int frequency=100;
  rigid2d::waypoints traj(way_p,max_Vx, max_Wz,1.0/frequency);
  rigid2d::Twist2D expected_pose,pose_error,ac_pose_error,d_pose_error,past_pose_error;
  ac_pose_error.Vx=0;
  ac_pose_error.Vy=0;
  ac_pose_error.Wz=0;
  initial_pose.Vx=ps.request.x;
  initial_pose.Vy=ps.request.y;
  initial_pose.Wz=ps.request.theta;
  ros::Subscriber odom_sub = n.subscribe("odom",1,odomBack);

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10,1);
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  marker.header.frame_id = "odom";
  marker.ns = "waypoints";
  marker.id = 0;
  marker.type=shape;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  past_pose_error.Vx=0;
  past_pose_error.Vy=0;
  past_pose_error.Wz=0;
  rigid2d::Twist2D Vb;
  ros::Rate r(frequency);
  double xtol=0.1,thetatol=0.1;
  while(ros::ok())
  {
    ros::spinOnce();
    if (flag_way==1)
    {
      if (resetCircle)
      {
        resetCircle=0;
        traj.reset();
      }
      int rot;
      rot=traj.getCircle();
      if (rot>0)
      {
        flag_way=0;
        continue;
      }
      expected_pose=traj.getpose();
      pose_error=traj.getTwistforError(actual_pose);
      if (pose_error.Vx>xtol)
      {
        pose_error.Vx=xtol;
      }else if(pose_error.Vx<-xtol)
      {
        pose_error.Vx=-xtol;
      }
      if (pose_error.Wz>thetatol)
      {
        pose_error.Wz=thetatol;
      }else if (pose_error.Wz<-thetatol)
      {
        pose_error.Wz=-thetatol;
      }
      //ROS_INFO("%f %f %f",pose_error.Vx,pose_error.Vy,pose_error.Wz);
      //rigid2d::Vector2D v=traj.gettarget();
      //ROS_INFO("tart x,y:%f %f %f %f %f",v.x,v.y,expected_pose.Vx,expected_pose.Vy,expected_pose.Wz);
      //ROS_INFO("tart x,y:%f %f %f %f %f",v.x,v.y,actual_pose.Vx,actual_pose.Vy,actual_pose.Wz);
      
      //ROS_INFO("Vb.w:%f x:%f y:%f theta:%f",Vb.Wz,expected_pose.Vx,expected_pose.Vy,expected_pose.Wz);
      //ROS_INFO("tart x,y:%f %f %f %f %f",v.x,v.y,Vb.Vx,Vb.Vy,Vb.Wz);
      // pose_error=expected_pose-actual_pose;
      //ROS_INFO("error:%f %f %f:",pose_error.Vx,pose_error.Vy,pose_error.Wz);
      d_pose_error=(pose_error-past_pose_error)*frequency;
      past_pose_error=pose_error;
      ac_pose_error+=pose_error/frequency;
      Vb=traj.nextWaypoint();
      Vb.Vx*=frequency;
      Vb.Wz*=frequency;

      Vb.Vx+=pose_error.Vx*kp_l+ac_pose_error.Vx*ki_l+d_pose_error.Vx*kd_l;
      Vb.Wz+=pose_error.Wz*kp_w+ac_pose_error.Wz*ki_w+d_pose_error.Wz*kd_w;
      cmd.linear.x=Vb.Vx;
      cmd.linear.y=0;
      cmd.linear.z=0;
      cmd.angular.x=0;
      cmd.angular.y=0;
      cmd.angular.z=Vb.Wz;
      //ROS_INFO("cmd %f",cmd.angular.z);
      cmd_pub.publish(cmd);
    }
    r.sleep();
    
  }
  

  return 0;

}
