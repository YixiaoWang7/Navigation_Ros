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

static double left_wheel_angle, right_wheel_angle;
static rigid2d::DiffDrive diffModel;
static ros::Publisher odom_pub, path_pub;
static ros::Time current_time, last_time;
static nav_msgs::Path path;

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

  // define the Odometry odom
  nav_msgs::Odometry odom;
  //the time should be equal to the joint state
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = pose_odom.Vx;
  odom.pose.pose.position.y = pose_odom.Vy;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = twist_odom.Vx;
  odom.twist.twist.linear.y = twist_odom.Vy;
  odom.twist.twist.angular.z = twist_odom.Wz;

  // publish the message
  odom_pub.publish(odom);

  // tf2 broadcaster
  static tf2_ros::TransformBroadcaster br;

  // transform
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = pose_odom.Vx;
  transformStamped.transform.translation.y = pose_odom.Vy;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation = odom_quat;
  // publish the transform
  br.sendTransform(transformStamped);

  // path
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.header.stamp = msg->header.stamp;
  poseStamped.header.frame_id = "odom";
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

  // set the private parameters.
  // note that the private parameters of joint state should be equal to the names in .xacro file

  // odom_frame_id: The name of the odometry tf frame
  // body_frame_id: The name of the body tf frame
  // left_wheel_joint: The name of the left wheel joint
  // right_wheel_joint: The name of the right wheel joint
  ros::NodeHandle np("~");
  np.setParam("odom_frame_id", "odom");
  np.setParam("body_frame_id", "base_link");
  np.setParam("left_wheel_joint", "left_wheel_axle");
  np.setParam("right_wheel_joint", "right_wheel_axle");
  
  // ready to publish
  ros::NodeHandle n;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 200);
  path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);

  // get the paramters of wheel and input it to diff model
  double wb,wr;
  n.getParam("wheel_radius",wr);
  n.getParam("wheel_base",wb);

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
  ros::Subscriber joint_state_sub=n.subscribe("joint_states",200,jointStateBack);

  ros::spin();
  return 0;
  
}