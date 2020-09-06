#include<iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include <ros/ros.h>
#include <gtest/gtest.h>

#include "rigid2d/rigid2d.hpp"

#include <geometry_msgs/Twist.h>
#include <nuturtlebot/WheelCommands.h>
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>

static ros::Publisher cmd_vel_pub, sensor_data_pub;
static ros::Subscriber wheel_cmd_sub, joint_states_sub;
static bool flag_wheel_state=1, flag_joint_state=1;
static double wr=0,wb=0,max_rot_v_motor=0;
static nuturtlebot::WheelCommands wc;
static std::string body_frame_id, left_wheel_joint, right_wheel_joint;
static double lp=0,lv=0,rp=0,rv=0;
static ros::Time nowjoint;

void wheelCmdBack(const nuturtlebot::WheelCommands::ConstPtr & msg)
{
  flag_wheel_state=0;
  wc.left_velocity=msg->left_velocity;
  wc.right_velocity=msg->right_velocity;
}

void jointStatesBack(const sensor_msgs::JointState::ConstPtr & msg)
{
  nowjoint=msg->header.stamp;
  lp=msg->position[0];
  rp=msg->position[1];
  lv=msg->velocity[0];
  rv=msg->velocity[1];
}

TEST(turtleinterfaceTest,cmdVelTest)
{
  geometry_msgs::Twist t;
  t.linear.x=0.1;
  t.linear.y=0;
  t.linear.z=0;
  t.angular.x=0;
  t.angular.y=0;
  t.angular.z=0;
  cmd_vel_pub.publish(t);
  ros::NodeHandle n;
  wheel_cmd_sub=n.subscribe("wheel_cmd",10,wheelCmdBack);
  while (flag_wheel_state)
  {
    ros::spinOnce();
  }
  ASSERT_NEAR(wc.left_velocity,round(0.1/wr/max_rot_v_motor*256),1.0e-5) << "translating: cmd_vel to wheel_cmd fails.";
  ASSERT_NEAR(wc.right_velocity,round(0.1/wr/max_rot_v_motor*256),1.0e-5) << "translating: cmd_vel to wheel_cmd fails.";

  t.linear.x=0;
  t.linear.y=0;
  t.linear.z=0;
  t.angular.x=0;
  t.angular.y=0;
  t.angular.z=1.5;
  cmd_vel_pub.publish(t);
  flag_wheel_state=1;
  while (flag_wheel_state)
  {
    ros::spinOnce();
  }
  ASSERT_NEAR(wc.left_velocity,-round(1.5*wb/wr/2/max_rot_v_motor*256),1.0e-5) << "rotating: cmd_vel to wheel_cmd fails.";
  ASSERT_NEAR(wc.right_velocity,round(1.5*wb/wr/2/max_rot_v_motor*256),1.0e-5) << "rotating: cmd_vel to wheel_cmd fails.";
  t.linear.x=0.05;
  t.linear.y=0;
  t.linear.z=0;
  t.angular.x=0;
  t.angular.y=0;
  t.angular.z=1;
  cmd_vel_pub.publish(t);
  flag_wheel_state=1;
  while (flag_wheel_state)
  {
    ros::spinOnce();
  }
  ASSERT_NEAR(wc.left_velocity,round((-wb/2.0*1.0+0.05)/wr/max_rot_v_motor*256),1.0e-5) << "translating and rotating: cmd_vel to wheel_cmd fails.";
  ASSERT_NEAR(wc.right_velocity,round((wb/2.0*1.0+0.05)/wr/max_rot_v_motor*256),1.0e-5) << "translating and rotating: cmd_vel to wheel_cmd fails.";

}

TEST(turtleinterfaceTest,sensorDataTest)
{
  nuturtlebot::SensorData sd1,sd2;
  ros::Time current_time,last_time;
  ros::Rate sle(10);
  last_time=ros::Time::now();
  sle.sleep();
  current_time=ros::Time::now();
  sd1.stamp=last_time;
  sd1.left_encoder=1000;
  sd1.right_encoder=2000;
  sd1.accelX=0;
  sd1.accelY=0;
  sd1.accelZ=0;
  sd1.gyroX=0;
  sd1.gyroY=0;
  sd1.gyroZ=0;
  sd1.magX=0;
  sd1.magY=0;
  sd1.magZ=0;
  sd1.battery_voltage=0.0;

  sd2.stamp=last_time;
  sd2.left_encoder=1000;
  sd2.right_encoder=2000;
  sd2.accelX=0;
  sd2.accelY=0;
  sd2.accelZ=0;
  sd2.gyroX=0;
  sd2.gyroY=0;
  sd2.gyroZ=0;
  sd2.magX=0;
  sd2.magY=0;
  sd2.magZ=0;
  sd2.battery_voltage=0.0;
  ros::NodeHandle n;
  joint_states_sub=n.subscribe("joint_states",10,jointStatesBack);
  ros::Duration dt;
  dt=current_time-last_time;

  while (flag_joint_state)
  {
    sensor_data_pub.publish(sd1);
    ros::spinOnce();
    if (nowjoint==last_time)
    {
      ASSERT_NEAR(lp,1000.0/4096*2*rigid2d::PI,1e-5) << "Sensor_data to joint_states fail.";
      ASSERT_NEAR(rp,2000.0/4096*2*rigid2d::PI,1e-5) << "Sensor_data to joint_states fail.";
      flag_joint_state=0;
    }
  }
  flag_joint_state=1;
  while (flag_joint_state)
  {
    sd2.stamp=current_time;
    sd2.left_encoder=sd2.left_encoder+2000;
    sd2.right_encoder=sd2.right_encoder+4000;
    sensor_data_pub.publish(sd2);
    ros::spinOnce();
    if (nowjoint!=last_time)
    {
      ASSERT_NEAR(lv,2000.0/4096*2*rigid2d::PI/dt.toSec(),1e-5) << "Sensor_data to joint_states fail.";
      ASSERT_NEAR(rv,4000.0/4096*2*rigid2d::PI/dt.toSec(),1e-5) << "Sensor_data to joint_states fail.";
      flag_joint_state=0;
    }
    current_time=current_time+dt;
  }


  

}









int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "turtle_interface_test");
  ros::NodeHandle nh;
  nh.getParam("wheel_radius",wr);
  nh.getParam("wheel_base",wb);
  nh.getParam("max_rot_v_motor",max_rot_v_motor);
  body_frame_id="base_link";
  left_wheel_joint="left_wheel_axle";
  right_wheel_joint="right_wheel_axle";

  cmd_vel_pub=nh.advertise<geometry_msgs::Twist>("cmd_vel",1,1);
  sensor_data_pub=nh.advertise<nuturtlebot::SensorData>("sensor_data",10,1);

  return RUN_ALL_TESTS();
}
