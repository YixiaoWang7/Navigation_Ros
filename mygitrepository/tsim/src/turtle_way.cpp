#include "ros/ros.h"
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
#include <rigid2d/diff_drive.hpp>


/**define several global variables:
PoseNow: the real-time pose of the turtle
pen_state: the paramters of set_pen service
lb_cor: the parameters of teleport_absolute service for reseting the turtle
*/

static turtlesim::Pose PoseNow;
static turtlesim::SetPen pen_state;
static turtlesim::TeleportAbsolute lb_cor;


/**subcribe the pose of the turtle from the topic*/
void PoseCallback(const turtlesim::Pose::ConstPtr& PoseMsg)
{
  PoseNow.x = PoseMsg->x;
  PoseNow.y = PoseMsg->y;
  PoseNow.theta = PoseMsg->theta;
  PoseNow.linear_velocity = PoseMsg->linear_velocity;
  PoseNow.angular_velocity = PoseMsg->angular_velocity;
}

int main(int argc, char **argv)
{
  //define the node called turtle_way
  ros::init(argc, argv, "turtle_way");
  ros::NodeHandle n;
  
  //define the variables
  //way_p: waypoints
  //single_p: used to generate five pent points
  //point: used to derive the points from paramters waypoint_x and waypoint_y
  //waypoint_x & waypoint_y: user defined parameters
  //NofP: number of the waypoints
  std::vector<rigid2d::Vector2D> way_p;
  rigid2d::Vector2D single_p, point;
  std::vector<double> waypoint_x, waypoint_y;  
  int NofP;

  //generate the five pent points coordinates and store them in waypoint_x and waypoint_y
  single_p.x=4;
  single_p.y=4;
  int side_length=4;
  waypoint_x.push_back(single_p.x);
  waypoint_y.push_back(single_p.y);
  for (int i=0;i<4;i++)
  {
    single_p.x+=side_length*cos(2*rigid2d::PI/5*i);
    single_p.y+=side_length*sin(2*rigid2d::PI/5*i);
    waypoint_x.push_back(single_p.x);
    waypoint_y.push_back(single_p.y);
  }
  
  //generate waypoints from parameters: waypoint_x and waypoint_y
  NofP=waypoint_x.size();
  if (NofP<2)
  {
    throw "the number of waypoints should be more than 1.";
  }
  for (int i=0;i<NofP;i++)
  {
    point.x=waypoint_x[i];
    point.y=waypoint_y[i];
    way_p.push_back(point);
  }

  //define the control frequency and the max velocity
  //frequency: command frequency
  //max_Vx: max linear velocity in xb axis
  //max_Wz: max angular velocity in zb axis
  //Vb: the twist obtained from the class waypoints,
  //    note that Vb is the actual displacement of diff model between two commands
  int frequency=60;
  double max_Vx=0.5, max_Wz=0.5;
  rigid2d::waypoints pent_traj(way_p,max_Vx, max_Wz,1.0/frequency);
  rigid2d::Twist2D Vb;
  

  //wait the services from turtlesim pkg to start
  ros::service::waitForService("/turtle1/set_pen");
  ros::service::waitForService("/turtle1/teleport_absolute");

  //set the pen_state parameter: red pen and the width is 3
  pen_state.request.r=255;
  pen_state.request.g=0;
  pen_state.request.b=0;
  pen_state.request.width=3;
  pen_state.request.off=1;

  //call the service: lift the pen
  ros::service::call("/turtle1/set_pen",pen_state);

  //set the teleport_abosulte parameter: left down corner of the pent
  lb_cor.request.x=way_p[0].x;
  lb_cor.request.y=way_p[0].y;
  lb_cor.request.theta=0.0;
  
  //teleport first and then put down the pen.
  ros::service::call("/turtle1/teleport_absolute",lb_cor);
  pen_state.request.off=0;
  ros::service::call("/turtle1/set_pen",pen_state);


  //subscribe the topic pose and store the pose data in PoseNow
  ros::Subscriber pos = n.subscribe<turtlesim::Pose>("turtle1/pose", 1000, PoseCallback);
  
  //define two publisher: cmd_vel to send command signal; pose_error to add the pose_error topic and send the signals
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Publisher poseerror_pub = n.advertise<tsim::PoseError>("pose_error", 1000);

  //define two kinds of msg type corresponding to the above publishers
  tsim::PoseError poseerror;
  
  //set the command frequency
  ros::Rate loop_rate(frequency);

  //expected_pose: the expeted pose of the diff model
  rigid2d::Twist2D expected_pose;

  //control loop
  while (ros::ok())
  {
    //before the command, get the expected pose.
    expected_pose=pent_traj.getpose();
    //compute the errors
    poseerror.x_error=fabs(expected_pose.Vx-PoseNow.x);
    poseerror.y_error=fabs(expected_pose.Vy-PoseNow.y);
    poseerror.theta_error=fabs(expected_pose.Wz-PoseNow.theta);
    if (poseerror.theta_error>rigid2d::PI) poseerror.theta_error=2*rigid2d::PI-poseerror.theta_error;
    poseerror_pub.publish(poseerror);   

    //cmd_V: command signal
    //       note that Vb is the actual displacemnt, so the command signal should be Vb multiplied by frequency.
    Vb=pent_traj.nextWaypoint();
    geometry_msgs::Twist cmd_v;
    cmd_v.linear.x=Vb.Vx*frequency;
    cmd_v.linear.y=Vb.Vy*frequency;
    cmd_v.linear.z=0;
    cmd_v.angular.x=0;
    cmd_v.angular.y=0;
    cmd_v.angular.z=Vb.Wz*frequency;
    //publish the command signal.
    cmd_pub.publish(cmd_v);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
