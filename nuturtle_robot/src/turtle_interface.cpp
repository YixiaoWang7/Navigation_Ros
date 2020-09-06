/// \file
/// \brief interface node. subscribe the sensor data from turtlebot and publish the joint states
///        subscribe the velocity command and publish the wheel velocity command 
///
/// PARAMETERS:
///     wheel_radius (double): wheel radius of the diff model
///     wheel_base (double): the distance between the centers of two wheels
///     max_rot_v_motor (double): max rotation velocity for the motor
///     max_trans_v (double): the max translation velocity of the robot
///     max_rot_v_robot (double): the max rotation velocity of the robot
///     encoder_ticks_per_rev (double): the number of ticks every rotation
///     /odometer/odom_frame_id (str): the name of the odom frame
///     /odometer/body_frame_id (str): the name of the body frame
///     /odometer/left_wheel_joint (str): the name of left wheel joint
///     /odometer/right_wheel_joint (str): the name of right wheel joint
/// PUBLISHES:
///     joint_states (sensor_msgs::JointState): the joint states of the diff model
///     wheel_cmd (nuturtlebot::WheelCommands): wheel velocity command sent to turtlebot from -256 to 256
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::Twist): velocity command sent to the robot, which is body twist
///     sensor_data (nuturtlebot::SensorData): sensor data from the turtlebot


#include <iosfwd> 
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>
#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>
#include <math.h>

static ros::Publisher wheel_cmd_pub, joint_states_pub;
static rigid2d::DiffDrive diffModel;
static double max_rot_v_motor=0,max_trans_v=0,max_rot_v_robot=0, last_joint_left=0, last_joint_right=0;
static int encoder_ticks_per_rev=0;
static double dt=0;
static ros::Time current_time, last_time;
static int joint_count=0;
static std::string odom_frame_id="odom",body_frame_id="base_link",left_wheel_joint="left_wheel_axle",right_wheel_joint="right_wheel_axle";

void cmdVelBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    rigid2d::Twist2D t;
    t.Wz=msg->angular.z;
    t.Vx=msg->linear.x;
    t.Vy=0;
    if(t.Wz>max_rot_v_robot)
    {
        t.Wz=max_rot_v_robot;
    }else if(t.Wz<-max_rot_v_robot)
    {
        t.Wz=-max_rot_v_robot;
    }
    if (t.Vx>max_trans_v)
    {
        t.Vx=max_trans_v;
    }else if (t.Vx<-max_trans_v)
    {
        t.Vx=-max_trans_v;
    }
    rigid2d::WheelVelocities wv;
    nuturtlebot::WheelCommands wv_cmd;
    wv=diffModel.twistToWheels(t);
    wv_cmd.left_velocity=round(wv.w_left/max_rot_v_motor*256.0);
    wv_cmd.right_velocity=round(wv.w_right/max_rot_v_motor*256.0);
    
    if (wv_cmd.left_velocity>256)
    {
        wv_cmd.left_velocity=256;
    }else if (wv_cmd.left_velocity<-256)
    {
        wv_cmd.left_velocity=-256;
    }
    if (wv_cmd.right_velocity>256)
    {
        wv_cmd.right_velocity=256;
    }else if (wv_cmd.right_velocity<-256)
    {
        wv_cmd.right_velocity=-256;
    }
    wheel_cmd_pub.publish(wv_cmd);
    
}
void sensorDataBack(const nuturtlebot::SensorData::ConstPtr& msg)
{
    current_time=msg->stamp;
    dt=(current_time-last_time).toSec();
    last_time=current_time;
    if (joint_count==0)
    {
        dt=0;
        joint_count=1;
    }
    double joint_left, joint_right, joint_left_v=0,joint_right_v=0;
    joint_left=msg->left_encoder/double(encoder_ticks_per_rev)*2*rigid2d::PI;
    joint_right=msg->right_encoder/double(encoder_ticks_per_rev)*2*rigid2d::PI;
    if (fabs(dt)>1.0e-11)
    {
        joint_left_v=(joint_left-last_joint_left)/dt;
        joint_right_v=(joint_right-last_joint_right)/dt;
    }
    last_joint_right=joint_right;
    last_joint_left=joint_left;
    sensor_msgs::JointState joint_pub;
    joint_pub.header.stamp=current_time;
    joint_pub.header.frame_id=body_frame_id;
    joint_pub.name={left_wheel_joint,right_wheel_joint};
    joint_pub.position={joint_left,joint_right};
    joint_pub.velocity={joint_left_v,joint_right_v};
    joint_states_pub.publish(joint_pub);

}


int main(int argc, char** argv){
  ros::init(argc, argv, "turtle_interface");

  ros::NodeHandle n;
  n.getParam("/odometer/odom_frame_id",odom_frame_id);
  n.getParam("/odometer/body_frame_id",body_frame_id);
  n.getParam("/odometer/left_wheel_joint",left_wheel_joint);
  n.getParam("/odometer/right_wheel_joint",right_wheel_joint);

  double wb,wr;
  n.getParam("wheel_radius",wr);
  n.getParam("wheel_base",wb);
  n.getParam("max_rot_v_motor",max_rot_v_motor);
  n.getParam("max_trans_v",max_trans_v);
  n.getParam("max_rot_v_robot",max_rot_v_robot);
  n.getParam("encoder_ticks_per_rev",encoder_ticks_per_rev);
  diffModel.setParam(wb,wr);

  last_time=ros::Time::now();
  joint_states_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  wheel_cmd_pub = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd",1);
  ros::Subscriber cmd_vel_sub=n.subscribe("cmd_vel",1,cmdVelBack);
  ros::Subscriber sensor_data_sub=n.subscribe("sensor_data",1,sensorDataBack);
  ros::spin();
  return 0;
  
}