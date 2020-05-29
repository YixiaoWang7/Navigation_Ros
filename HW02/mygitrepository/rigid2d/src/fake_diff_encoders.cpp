/// \file
/// \brief fake encoders. subscribe the velocity command, predict 
///        the joint states of the diff model based on velocity 
///        command and publish the joint states
///
/// PARAMETERS:
///     wheel_radius (double): wheel radius of the diff model
///     wheel_base (double): the distance between the centers of two wheels
///     /odometer/odom_frame_id (str): the name of the odom frame
///     /odometer/body_frame_id (str): the name of the body frame
///     /odometer/left_wheel_joint (str): the name of left wheel joint
///     /odometer/right_wheel_joint (str): the name of right wheel joint
/// PUBLISHES:
///     joint_states (sensor_msgs::JointState): the expected joint states
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::Twist): velocity command sent to the robot, which is body twist


#include <iosfwd> 
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

static std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
static rigid2d::DiffDrive diffModel;
static ros::Publisher jointstates_pub;
static ros::Time current_time, last_time;
static rigid2d::Twist2D Vb={0,0,0};

void controlSignalBack(const geometry_msgs::Twist::ConstPtr& msg)
{
  current_time=ros::Time::now();

  // calculate the time interval
  double dt;
  dt=(current_time-last_time).toSec();
  last_time=current_time;

  // calculate the twist during the time interval/or call it as displacment
  Vb.Vx*=dt;
  Vb.Vy*=dt;
  Vb.Wz*=dt;

  // define jointstates and jointvelocities
  rigid2d::WheelVelocities jointstates, jointvelocities;

  // obtain the joint difference during the time interval
  jointvelocities=diffModel.twistToWheels(Vb);

  // renew the diff model
  diffModel.feedforward(Vb);

  // obtain the joint state
  jointstates=diffModel.getJointStateFeedforward();

  // define the JointState class
  // note the actual joint velocity should be joint difference / dt.
  // for the visualization, publish an empty joint state for ball_caster_joint
  sensor_msgs::JointState joint_pub;
  joint_pub.header.stamp=current_time;
  joint_pub.header.frame_id=body_frame_id;
  joint_pub.name={left_wheel_joint,right_wheel_joint};
  joint_pub.position={jointstates.w_left,jointstates.w_right};
  joint_pub.velocity={jointvelocities.w_left/dt,jointvelocities.w_right/dt};

  // publish JointState
  jointstates_pub.publish(joint_pub);

  // renew the twist
  Vb.Vx=msg->linear.x;
  Vb.Vy=msg->linear.y;
  Vb.Wz=msg->angular.z;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_diff_encoders");
  ros::NodeHandle n;

  // ready to publish the topic joint_state
  jointstates_pub=n.advertise<sensor_msgs::JointState>("joint_states", 10);


  // read the parameter of differential velocity model
  double wb,wr;
  n.getParam("wheel_radius",wr);
  n.getParam("wheel_base",wb);

  // read the names defined in odometer node
  n.getParam("/odometer/odom_frame_id",odom_frame_id);
  n.getParam("/odometer/body_frame_id",body_frame_id);
  n.getParam("/odometer/left_wheel_joint",left_wheel_joint);
  n.getParam("/odometer/right_wheel_joint",right_wheel_joint);

  // set the initial configuration and the wheel_base & wheel_radius
  rigid2d::Twist2D initial_configuration;
  initial_configuration.Vx=0;
  initial_configuration.Vy=0;
  initial_configuration.Wz=0;

  // set the parameters for the diff model
  diffModel.setParam(wb,wr);

  // set the position and reset the other parameters to be zeros
  diffModel.reset(initial_configuration);

  // initial the time, note that it does not matter because the twist of the first point will be zero.
  // subsribe the topic /turtle1/cmd_vel
  last_time=ros::Time::now();
  ros::Subscriber cmd_sub=n.subscribe("cmd_vel",10,controlSignalBack);

  ros::spin();
  return 0;
}