#include "ros/ros.h"
#include "std_msgs/String.h"
#include <turtlesim/Pose.h>
#include "geometry_msgs/Twist.h"
#include <tsim/PoseError.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>
#include <std_srvs/Empty.h>
#include <rigid2d/rigid2d.hpp>

/**define several global variables:
PoseNow: the real-time pose of the turtle
pen_state: the paramters of set_pen service
lb_cor: the parameters of teleport_absolute service for reseting the turtle
*/
static turtlesim::Pose PoseNow;
static turtlesim::SetPen pen_state;
static turtlesim::TeleportAbsolute lb_cor;
static int stage_count=2;

/**traject reset function to reset the turtle*/
bool TrajReset(std_srvs::Empty::Request &req,  std_srvs::Empty::Response &res)
{
  ros::NodeHandle n;
  /**get the parameter of the left down cornor of the rectangle*/
  float x,y;
  n.getParam("x",x);
  n.getParam("y",y);

  /**lift the pen*/
  pen_state.request.off=1;
  ros::service::call("/turtle1/set_pen",pen_state);

  /**teleport the turtle*/
  ros::service::call("/turtle1/teleport_absolute",lb_cor);

  /**put down the pen*/
  pen_state.request.off=0;
  ros::service::call("/turtle1/set_pen",pen_state);

  /**reset the stage_count.*/
  stage_count=2;
  return true;
}

/**subcribe the pose of the turtle from the topic*/
void PoseCallback(const turtlesim::Pose::ConstPtr& PoseMsg)
{
  PoseNow.x = PoseMsg->x;
  PoseNow.y = PoseMsg->y;
  PoseNow.theta = PoseMsg->theta;
  PoseNow.linear_velocity = PoseMsg->linear_velocity;
  PoseNow.angular_velocity = PoseMsg->angular_velocity;

  ROS_INFO("Pose of the turtle:");
  ROS_INFO("x: [%f]", PoseNow.x);
  ROS_INFO("y: [%f]", PoseNow.y);
  ROS_INFO("theta: [%f]", PoseNow.theta);
  ROS_INFO("linear_velocity: [%f]", PoseNow.linear_velocity);
  ROS_INFO("angular_velocity: [%f]\n", PoseNow.angular_velocity);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle n;
  
  /**get parameters from .yaml file*/
  int x, y, width, height, trans_vel, rot_vel, frequency;
  n.getParam("x",x);
  n.getParam("y",y);
  n.getParam("width",width);
  n.getParam("height",height);
  n.getParam("trans_vel",trans_vel);
  n.getParam("rot_vel",rot_vel);
  n.getParam("frequency",frequency);

  /**ROS_INFO the parameters*/
  ROS_INFO("x: [%d]", x);
  ROS_INFO("y: [%d]", y);
  ROS_INFO("width: [%d]", width);
  ROS_INFO("height: [%d]", height);
  ROS_INFO("trans_vel: [%d]", trans_vel);
  ROS_INFO("rot_vel: [%d]", rot_vel);
  ROS_INFO("frequency: [%d]", frequency);

  /**wait the services from turtlesim pkg to start*/
  ros::service::waitForService("/turtle1/set_pen");
  ros::service::waitForService("/turtle1/teleport_absolute");

  /**set the pen_state parameter: red pen and the width is 3*/
  pen_state.request.r=255;
  pen_state.request.g=0;
  pen_state.request.b=0;
  pen_state.request.width=3;
  pen_state.request.off=1;

  /**call the service: lift the pen*/
  ros::service::call("/turtle1/set_pen",pen_state);

  /**set the teleport_abosulte parameter: left down corner of the rectangle*/
  lb_cor.request.x=x;
  lb_cor.request.y=y;
  lb_cor.request.theta=0.0;
  
  /**teleport first and then put down the pen.*/
  ros::service::call("/turtle1/teleport_absolute",lb_cor);
  pen_state.request.off=0;
  ros::service::call("/turtle1/set_pen",pen_state);

  /**after the turtle gets to the left down corner, start the reset service.*/
  ros::ServiceServer service = n.advertiseService("traj_reset", TrajReset);

  /**subscribe the topic pose and store the pose data in PoseNow*/
  ros::Subscriber pos = n.subscribe<turtlesim::Pose>("turtle1/pose", 1000, PoseCallback);
  
  /** define two publisher: cmd_vel to send command signal; pose_error to add the pose_error topic and send the signals*/
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Publisher poseerror_pub = n.advertise<tsim::PoseError>("pose_error", 1000);

  /**define two kinds of msg type corresponding to the above publishers*/
  geometry_msgs::Twist cmd_v;
  tsim::PoseError poseerror;

  /**the velocities are all zeros except Vx and Wz.*/
  cmd_v.linear.y=0.0;
  cmd_v.linear.z=0.0;
  cmd_v.angular.x=0.0;
  cmd_v.angular.y=0.0;
  
  /**command frequency*/
  ros::Rate loop_rate(frequency);

  /**
  define the parameters:
    count: the command count
    stage_count: distinguish every edges of the rectangle
    target_x&target_y: the goal pose the turtle should move towards
    linear_tol&angular_tol: the accuracy of the linear and angular position
  */
  int count=0;

  int target_x;
  int target_y;
  float linear_tol=0.01;
  float angular_tol=0.01;

  /**control loop*/
  while (ros::ok())
  {
  /**stage_count&target point*/
  if (stage_count==1)
  {
    target_x=x;
    target_y=y;
  }
  else if (stage_count==2)
  {
    target_x=x+width;
    target_y=y;
  }
  else if (stage_count==3)
  {
    target_x=x+width;
    target_y=y+height;
  }
  else if (stage_count==4)
  {
    target_x=x;
    target_y=y+height;
  }

  /**compute the errors*/
  float thetapose, theta_error, theta_error0, x_error, y_error;
  float thetanow=PoseNow.theta;
  x_error=target_x-PoseNow.x;
  y_error=target_y-PoseNow.y;

  /**theta_error is a little difficult to compute. First, transform the negative value into the positive value, or in other words, map theta from (-PI,PI) into (0,2*PI). Second, the absolute difference should belong to (0,PI). As to the definition into the rotation direction, the method is shown below. The goal is to let the turtle rotate the minimal angle to reach the target angle.*/
 
  /**atan2 will provide the angle belonging to (-PI,PI).*/
  thetapose = atan2(y_error,x_error);
  /**map the angle from (-PI,PI) into (0,2*PI).*/
  if (thetapose<0) thetapose=2*rigid2d::PI+thetapose;
  if (thetanow<0) thetanow=2*rigid2d::PI+thetanow;
  theta_error0 = thetapose-thetanow;
  theta_error = abs(theta_error0);
  /**absolute angle_error should belong to (0,PI).*/
  if (theta_error>rigid2d::PI) theta_error=2*rigid2d::PI-theta_error;
  poseerror.x_error=abs(x_error);
  poseerror.y_error=abs(y_error);
  poseerror.theta_error=theta_error;
  poseerror_pub.publish(poseerror);   

  /**the control strategy is to rotate first and then translation. The aim of the strategy is to minimize the error between real-time positionn and the target position. The proof is that if there is any linear velocity when the orientation of the turtle is not towards the target point, the turtle must deviating the trajectory, the line connecting the real-time posisiton and the target point.*/

  /**If the turtle reaches the target point, no velocity.*/
  if (x_error*x_error+y_error*y_error<linear_tol)
  {
    cmd_v.angular.z=0;
    cmd_v.linear.x=0;
    stage_count++;

    /**The turtle will move along the rectangle constantly. So the stage_count should be renewed.*/
    if (stage_count>4) stage_count=1;
  }
  
  /**If the theta_error is small enough, go in a straight line.*/
  else if (theta_error<angular_tol)
  {
    cmd_v.linear.x=trans_vel;
  }
  else
  {

    /**If the theta_error is not small enough, Vx should be zero and the turtle rotates*/
    cmd_v.linear.x=0;
    if (theta_error0>=0&&theta_error0<rigid2d::PI)
    {
      cmd_v.angular.z=rot_vel;
    }
    else if (theta_error0>=rigid2d::PI)
    {
      cmd_v.angular.z=-rot_vel;
    }
    else if (theta_error0<=0&&theta_error0>-rigid2d::PI)
    {
      cmd_v.angular.z=-rot_vel;
    }
    else if (theta_error0<=-rigid2d::PI)
    {
      cmd_v.angular.z=rot_vel;
    } 
  }

  /**publish the command signal.*/
  cmd_pub.publish(cmd_v);
  ros::spinOnce();
  loop_rate.sleep();
  ++count;
  }

  return 0;
}
