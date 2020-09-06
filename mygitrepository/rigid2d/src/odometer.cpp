/// \file
/// \brief odometry. subscribe the velocity command, calculate the odometry 
///        and publish the odometry and its path 
///
/// PARAMETERS:
///     public:
///     wheel_radius (double): wheel radius of the diff model
///     wheel_base (double): the distance between the centers of two wheels
///
///     private:
///     odom_frame_id (str): the name of the odom frame
///     body_frame_id (str): the name of the body frame
///     left_wheel_joint (str): the name of left wheel joint
///     right_wheel_joint (str): the name of right wheel joint
/// PUBLISHES:
///     odom (nav_msgs::Odometry): the odometry according to joint states
///     trajectory (nav_msgs::Path): the path according to joint states
/// SUBSCRIBES:
///     joint_states (sensor_msgs::JointState): the joint states of the diff model
/// SERVICES:
///     set_pose (rigid2d::set_pose): teleport the odometry to the target pose and reset the odometry


#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rigid2d/set_pose.h>

static double left_wheel_angle, right_wheel_angle;
static rigid2d::DiffDrive diffModel;
static ros::Publisher odom_pub, path_pub;
static ros::Time current_time, last_time;
static nav_msgs::Path path;
static std::string odom_frame_id="odom",body_frame_id="base_link",left_wheel_joint="left_wheel_axle",right_wheel_joint="right_wheel_axle";
static nav_msgs::Odometry odom;
static geometry_msgs::TransformStamped transformStamped;
static geometry_msgs::PoseStamped poseStamped;
static double left_wheel_angle_of=0,right_wheel_angle_of=0;
static bool flag_set=0;

bool setPose(rigid2d::set_pose::Request  &req,rigid2d::set_pose::Response &res)
{
  rigid2d::Twist2D t;
  t.Vx=req.x;
  t.Vy=req.y;
  t.Wz=req.theta;
  diffModel.set_pose(t);
  flag_set=1;
  return true;
}


void jointStateBack(const sensor_msgs::JointState::ConstPtr& msg)
{
  // define the joint state according to the JointState.name
  //   for the fake_encoders only publish two joint states(the ball can be ignored),
  //   just judge whether the first JointState is left or not is enough.
  if (msg->name[0]=="left_wheel_axle")
  {
    left_wheel_angle=msg->position[0];
    right_wheel_angle=msg->position[1];
  }else
  {
    left_wheel_angle=msg->position[1];
    right_wheel_angle=msg->position[0];
  }
  if (flag_set==1)
  {
    left_wheel_angle_of=left_wheel_angle;
    right_wheel_angle_of=right_wheel_angle;
    flag_set=0;
  }
  left_wheel_angle-=left_wheel_angle_of;
  right_wheel_angle-=right_wheel_angle_of;

  // update the odometry based on joint states
  diffModel.updateOdometry(left_wheel_angle,right_wheel_angle);

  // get the pose of updated odometry
  rigid2d::Twist2D pose_odom;
  pose_odom=diffModel.pose();

  // get the twist of updated odometry
  rigid2d::Twist2D twist_odom;
  twist_odom=diffModel.wheelsToTwist(diffModel.wheelVelocities());

  // calculate the time interval between two JointStates
  double dt;
  current_time=msg->header.stamp;
  dt=(current_time-last_time).toSec();
  last_time=current_time;

  // note the value from diff_drive is the difference, not the velocity
  // velocity should be difference/dt
  if(fabs(dt)>1.0e-4)
  {
    twist_odom.Vx/=dt;
    twist_odom.Vy/=dt;
    twist_odom.Wz/=dt;
  }else
  {
    twist_odom.Vx=0;
    twist_odom.Vy=0;
    twist_odom.Wz=0;
    ROS_INFO("time interval for odometry is too small.");
  }
  


  // get the orientation of the model
  tf2::Quaternion q;
  q.setRPY(0, 0, pose_odom.Wz);
  geometry_msgs::Quaternion odom_quat;
  odom_quat.x=q.x();
  odom_quat.y=q.y();
  odom_quat.z=q.z();
  odom_quat.w=q.w();

  //the time should be equal to the joint state
  odom.header.stamp = msg->header.stamp;
  

  // set the position
  odom.pose.pose.position.x = pose_odom.Vx;
  odom.pose.pose.position.y = pose_odom.Vy;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity

  odom.twist.twist.linear.x = twist_odom.Vx;
  odom.twist.twist.linear.y = twist_odom.Vy;
  odom.twist.twist.angular.z = twist_odom.Wz;

  // publish the message
  odom_pub.publish(odom);

  // tf2 broadcaster
  static tf2_ros::TransformBroadcaster br;

  // transform
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.transform.translation.x = pose_odom.Vx;
  transformStamped.transform.translation.y = pose_odom.Vy;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = odom_quat;
  // publish the transform
  br.sendTransform(transformStamped);

  // path

  poseStamped.header.stamp = msg->header.stamp;
  poseStamped.pose.position.x = pose_odom.Vx;
  poseStamped.pose.position.y = pose_odom.Vy;
  poseStamped.pose.position.z = 0.0;
  poseStamped.pose.orientation = odom_quat;
  path.poses.push_back(poseStamped);
  // publish the trajectory
  path_pub.publish(path);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometer");
  ros::NodeHandle np("~");
  np.getParam("odom_frame_id", odom_frame_id);
  np.getParam("body_frame_id", body_frame_id);
  np.getParam("left_wheel_joint", left_wheel_joint);
  np.getParam("right_wheel_joint", right_wheel_joint);
  
  // ready to publish
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);

  // get the paramters of wheel and input it to diff model
  double wb,wr;
  n.getParam("/wheel_radius",wr);
  n.getParam("/wheel_base",wb);

  // set the diff model
  rigid2d::Twist2D initial_configuration;
  initial_configuration.Vx=0;
  initial_configuration.Vy=0;
  initial_configuration.Wz=0;
  diffModel.setParam(wb,wr);
  diffModel.reset(initial_configuration);

  path.header.stamp=ros::Time::now();
  path.header.frame_id="odom";

  // subscribe the joint_states
  // note the last_time initialization can be random, because the the first joint_states is zero.
  last_time=ros::Time::now();
  ros::Subscriber joint_state_sub=n.subscribe("joint_states",1,jointStateBack);

  // add the service set_pose
  ros::ServiceServer service = n.advertiseService("set_pose", setPose);

  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = body_frame_id;
  transformStamped.header.frame_id = odom_frame_id;
  transformStamped.child_frame_id = body_frame_id;
  poseStamped.header.frame_id = odom_frame_id;

  ros::spin();
  return 0;
  
}