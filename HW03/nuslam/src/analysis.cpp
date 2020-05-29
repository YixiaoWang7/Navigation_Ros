/// \file
/// \brief analysis node. subscribe the velocity command, calculate the odometry 
///        and publish the odometry and its path 
///
/// PARAMETERS:
///     private:
///     noiseVariance (double): variance of Gaussian noise
/// PUBLISHES:
///     landmarks (nuslam::TurtleMap): the positions of landmarks in the world frame
/// SERVICES:
///     /gazebo/get_world_properties (gazebo_msgs::GetWorldProperties): call the service to get the names of models
///     /gazebo/get_model_state (gazebo_msgs::GetModelState): call the service to get the positions of models


#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <nuslam/TurtleMap.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

#include<random>
 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }


int main(int argc, char** argv){
  ros::init(argc, argv, "analysis");
  ros::NodeHandle n;

  // publish the landmark position in world frames
  ros::Publisher landmarksPub=n.advertise<nuslam::TurtleMap>("landmarks",1);

  // get the parameters in the gazebo world including the positions of the landmarks
  ros::ServiceClient NameClient = n.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
  ros::ServiceClient StatesClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  // get the name of models in the gazebo world
  gazebo_msgs::GetWorldProperties Name;
  while(ros::ok())
  {
    if (NameClient.call(Name))
    {
      break;
    }else
    {
      ROS_INFO("fail to call /gazebo/get_world_properties.");
    }
    ros::Duration(0.5).sleep();
  }

  // get the name of landmarks
  std::vector<std::string> landmarksName;
  std::string temp;
  for (int i=0;i<int(Name.response.model_names.size());i++)
  {
    temp=Name.response.model_names[i];
    if (int(temp.size())>7)
    {
      if (temp.substr(0,8)=="cylinder")
      {
        landmarksName.push_back(temp);

      }
    }
    temp.clear();
  }
  temp.shrink_to_fit();

  // get the positions of landmarks
  gazebo_msgs::GetModelState states;
  nuslam::TurtleMap landmarks;
  for (int i=0;i<int(landmarksName.size());i++)
  {
    states.request.model_name=landmarksName[i];
    // states.request.relative_entity_name=landmarksBody[i];
    while (ros::ok())
    {
      if (StatesClient.call(states))
      {
        landmarks.x.push_back(states.response.pose.position.x);
        landmarks.y.push_back(states.response.pose.position.y);
        landmarks.radius.push_back(0.035);
        break;
      }else
      {
        ROS_INFO("fail to call /gazebo/get_model_state");
      }
    }
  }

  // add Gaussian noise and publish
  nuslam::TurtleMap landmarksNoisy;
  landmarksNoisy=landmarks;
  double noiseVariance=0.01;
  ros::param::get("~noiseVariance",noiseVariance);
  std::normal_distribution<> d(0, noiseVariance);
  while(ros::ok())
  {
    for (int i=0;i<int(landmarks.x.size());i++)
    {
      landmarksNoisy.x[i]=landmarks.x[i]+d(get_random());
      landmarksNoisy.y[i]=landmarks.y[i]+d(get_random());

    }
    landmarksPub.publish(landmarksNoisy);
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  return 0;
  
}