/// \file
/// \brief slam node. debug model or nondebug model. 
///        and publish transform and path of groundtruth and slam result.
///
/// PARAMETERS:
///     private:
///     debug (str): true or false. debug model on or off
///     parent_frame_id (str): the name of the parent frame. map frame in this case
///     body_frame_id (str): the name of the child frame.
///     detectRadius (double): detect radius for landmarks.
///     data_association_method (int): data association method: 1 or 2
///     associatedDis (double): Method 1: the Hamming distance. same landmark within this distance
///     newDis (double): Method 1: the Hamming distance. new landmark out of this distance
///     corvarianceI (int): Method 2: 0 or 1. 1 means the covariance matrix for Method 2 is identity.
///     threshold (double): Method 2: same landmark within this threshold.
/// PUBLISHES:
///     slamTrajectory (nav_msgs::Path): slam result path
///     groundTrajectory (nav_msgs::Path): groundtruth path
///     MapIndexVisual (nuslam::MapIndexVisual): if debug = true, this topic will be published. 
///                                              Publish the index of detected landmarks within
///                                              the detectRadius.
/// SUBSCRIBES:
///     landmarks (nuslam::TurtleMap): the positions of landmarks
///     cmd_vel (geometry_msgs::Twist): the body twist to drive the model
/// SERVICES:
///     /gazebo/set_model_state (gazebo_msgs::SetModelState): call the service to initial the pose of model
///     /gazebo/get_model_state (gazebo_msgs::GetModelState): call the service to get groundtruth


#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <rigid2d/rigid2d.hpp>
#include <ekf/ekf.hpp>
#include <rigid2d/diff_drive.hpp>
#include <nav_msgs/Path.h>
#include <nuslam/TurtleMap.h>
#include <nuslam/MapIndexVisual.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

static rigid2d::DiffDrive diffModel;


// get the landmarks position
// Note in debug model, landmark position is in the map frame.
// in nondebug model, landmark position is in the base_scan frame.
static nuslam::TurtleMap::ConstPtr r=nullptr;
void landmarksBack(const nuslam::TurtleMap::ConstPtr &msg)
{
  r=msg;
}


// get current cmd_vel u. 
// Note in ekf, the input command should be the previous command.
static Eigen::Vector3f u;
void cmdBack(const geometry_msgs::Twist::ConstPtr &cmdptr)
{
  u(0)=cmdptr->linear.x;
  u(1)=cmdptr->linear.y;
  u(2)=cmdptr->angular.z;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "slam");
  ros::NodeHandle n;

  // get private parameter debug, default = true.
  std::string debug="true";
  ros::param::get("~debug",debug);

  // subscribe two topics: landmarks and cmd_vel
  ros::Subscriber landmarksSub=n.subscribe("landmarks",1,landmarksBack);
  ros::Subscriber cmdSub=n.subscribe("cmd_vel",1,cmdBack);

  // call /gazebo/get_model_state service to get the groundtruth
  // call /gazebo/set_model_state service to initialize the initial pose of model
  ros::ServiceClient StatesClient = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient InitialClient = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // model name or robot name
  std::string robotName;
  robotName="nuturtle";

  // initialize the initial pose of the robot
  gazebo_msgs::SetModelState setState;
  gazebo_msgs::ModelState initialState; 
  initialState.model_name=robotName;
  initialState.pose.position.x=0;
  initialState.pose.position.y=0;
  initialState.pose.position.z=0.073;
  initialState.pose.orientation.x=0;
  initialState.pose.orientation.y=0;
  initialState.pose.orientation.z=0;
  initialState.pose.orientation.w=1;
  initialState.twist.linear.x=0;
  initialState.twist.linear.y=0;
  initialState.twist.linear.z=0;
  initialState.twist.angular.x=0;
  initialState.twist.angular.y=0;
  initialState.twist.angular.z=0;
  setState.request.model_state=initialState;
  while(ros::ok())
  {
    if (InitialClient.call(setState))
    {
      if (setState.response.success)
      {
        break;
      }else
      {
        ROS_INFO("fail to set /gazebo/set_model_state for nuturtle initialization.");
      }
      
    }else
    {
      ROS_INFO("fail to call /gazebo/set_model_state for nuturtle initialization.");
    }
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  
  // initialize the observationIndex and observation
  // which means the landmarks topic has messages, r!=nullptr. 
  std::vector<int> index, observationIndex;
  std::vector<Eigen::Vector2f> observation;
  while(ros::ok())
  {
    if (r!=nullptr)
    {
      break;
    }
    ros::spinOnce();
  }

  // initialize the trajectory and transform tree.
  std::string parent_frame_id="map",body_frame_id="base_link";
  ros::param::get("~parent_frame_id",parent_frame_id);
  ros::param::get("~body_frame_id",body_frame_id);

  // initialize the path
  nav_msgs::Path slamPath;
  nav_msgs::Path groundPath;
  geometry_msgs::PoseStamped poseStamped;
  ros::Publisher slamPathPub = n.advertise<nav_msgs::Path>("slamTrajectory",1, true);
  ros::Publisher groundPathPub = n.advertise<nav_msgs::Path>("groundTrajectory",1, true);
  poseStamped.header.frame_id=parent_frame_id;
  poseStamped.header.stamp=ros::Time::now();
  poseStamped.pose.position.x=0;
  poseStamped.pose.position.y=0;
  poseStamped.pose.position.z=0;
  poseStamped.pose.orientation.x=0;
  poseStamped.pose.orientation.y=0;
  poseStamped.pose.orientation.z=0;
  poseStamped.pose.orientation.w=1;
  slamPath.header.frame_id=parent_frame_id;
  slamPath.header.stamp=poseStamped.header.stamp;
  slamPath.poses.push_back(poseStamped);
  groundPath.header.frame_id=parent_frame_id;
  groundPath.header.stamp=poseStamped.header.stamp;
  groundPath.poses.push_back(poseStamped);

  // initialize the transform
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped truthtransformStamped;

  // slam transform
  transformStamped.header.frame_id=parent_frame_id;
  transformStamped.header.stamp=ros::Time::now();
  transformStamped.child_frame_id=body_frame_id;
  transformStamped.transform.translation.x=0;
  transformStamped.transform.translation.y=0;
  transformStamped.transform.translation.z=0;
  transformStamped.transform.rotation.x=0;
  transformStamped.transform.rotation.y=0;
  transformStamped.transform.rotation.z=0;
  transformStamped.transform.rotation.w=1;

  // groundtruth transform
  truthtransformStamped.header.frame_id=parent_frame_id;
  truthtransformStamped.header.stamp=transformStamped.header.stamp;
  truthtransformStamped.child_frame_id="groundTruth";
  truthtransformStamped.transform.translation.x=0;
  truthtransformStamped.transform.translation.y=0;
  truthtransformStamped.transform.translation.z=0;
  truthtransformStamped.transform.rotation.x=0;
  truthtransformStamped.transform.rotation.y=0;
  truthtransformStamped.transform.rotation.z=0;
  truthtransformStamped.transform.rotation.w=1;

  // ekf class
  ekf::ekfDiffLaser ekfCal;

  // approximate time interval to publish the topic
  double timeInterval=0.01;

  // groundtruth (x,y,theta)
  Eigen::VectorXf poseGroundtruth(3);

  if (debug=="true")
  {
    // pub the index of detected landmarks within the detectRadius.
    nuslam::MapIndexVisual visualIndex;
    ros::Publisher MapIndexVisualPub=n.advertise<nuslam::MapIndexVisual>("MapIndexVisual",1);

    // analysis node must publish >0 landmarks.
    while(ros::ok())
    {
      if (int(r->x.size())==0)
      {
        ROS_INFO("debug model publishes null landmarks. Location: analysis node");
      }else
      {
        for(int i=0;i<int(r->x.size());i++)
        {
          index.push_back(i);
        }
        break;
      }
      ros::spinOnce();
    }


    // for transformation between Quaternion and RPY
    tf2::Quaternion q;
    geometry_msgs::Quaternion geometryQuat;
    tf2::Matrix3x3 m;
    
    // first detect whether the gazebo starts properly, which means ground truth can be obtained.
    // and then initialize poseGroundtruth.
    gazebo_msgs::GetModelState groundtruth;
    while(ros::ok())
    {
      groundtruth.request.model_name=robotName;
      if (StatesClient.call(groundtruth))
      {
        poseGroundtruth(0)=groundtruth.response.pose.position.x;
        poseGroundtruth(1)=groundtruth.response.pose.position.y;
        q.setX(groundtruth.response.pose.orientation.x);
        q.setY(groundtruth.response.pose.orientation.y);
        q.setZ(groundtruth.response.pose.orientation.z);
        q.setW(groundtruth.response.pose.orientation.w);
        m.setRotation(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        poseGroundtruth(2)=yaw;
        // ROS_INFO("%f %f %f",poseGroundtruth(0),poseGroundtruth(1),poseGroundtruth(2));
        break;
      }else
      {
        ROS_INFO("fail to call /gazebo/get_model_state for nuturtle(groudtruth).");
      }
      ros::spinOnce();
    }

    // dt is a more accurate time interval between two iterations calculated by ros::Time.
    double dt=timeInterval;
    ros::Time currentTime,lastTime;
    currentTime=ros::Time::now();
    lastTime=ros::Time::now();

    // obTemp is (r,theta). consistent with laser data
    // it will be calculated by landmarks position in world frame and groundtruth
    Eigen::Vector2f obTemp;


    // k means iteration k;k+1 means now
    // so uk record the previous command input.
    Eigen::Vector3f uk;
    uk(0)=0;
    uk(1)=0;
    uk(2)=0;

    // slamPose is the pose calculated by ekf
    Eigen::Vector3f slamPose;

    // detectRadius
    double detectRadius=1;
    ros::param::get("~detectRadius",detectRadius);

    // ekf
    while(ros::ok())
    { 
      // if the service call succeed, go ahead; otherwise, use the previous state.
      //get the current observation and record current time
      ros::spinOnce();
      currentTime=ros::Time::now();
      if (StatesClient.call(groundtruth))
      {
        poseGroundtruth(0)=groundtruth.response.pose.position.x;
        poseGroundtruth(1)=groundtruth.response.pose.position.y;
        q.setX(groundtruth.response.pose.orientation.x);
        q.setY(groundtruth.response.pose.orientation.y);
        q.setZ(groundtruth.response.pose.orientation.z);
        q.setW(groundtruth.response.pose.orientation.w);
        m.setRotation(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        poseGroundtruth(2)=yaw;
        // ROS_INFO("%f %f %f",poseGroundtruth(0),poseGroundtruth(1),poseGroundtruth(2));
      }

      // calculate the observations based on landmark positions with Gaussian noise
      for (int i=0;i<int(r->x.size());i++)
      {
        double dx=0,dy=0;
        dx=r->x[i]-poseGroundtruth(0);
        dy=r->y[i]-poseGroundtruth(1);
        obTemp(0)=sqrt(pow(dx,2)+pow(dy,2));
        if (obTemp(0)<detectRadius)
        {
          obTemp(1)=atan2(dy,dx)-poseGroundtruth(2);
          observation.push_back(obTemp);
          observationIndex.push_back(i);
          visualIndex.index.push_back(i);
          // ROS_INFO("%f %f %d",obTemp(0),obTemp(1),i);
        }
      }
      MapIndexVisualPub.publish(visualIndex);
      visualIndex.index.clear();
      visualIndex.index.shrink_to_fit();


      // calculate the actual time interval
      dt=(currentTime-lastTime).toSec();

      // call ekf to calculate the current pose based on previous command
      slamPose=ekfCal.ekfCall(observationIndex, observation,uk*dt);

      // free memory
      observationIndex.clear();
      observationIndex.shrink_to_fit();
      observation.clear();
      observation.shrink_to_fit();

      // ROS_INFO("ground %f %f %f",poseGroundtruth(0),poseGroundtruth(1),poseGroundtruth(2));
      // ROS_INFO("slam %f %f %f",slamPose(0),slamPose(1),slamPose(2));
      // ROS_INFO("dt %f",dt);

      //path groundtruth
      geometryQuat.x=q.x();
      geometryQuat.y=q.y();
      geometryQuat.z=q.z();
      geometryQuat.w=q.w();
      poseStamped.header.stamp=currentTime;
      poseStamped.pose.position.x=poseGroundtruth(0);
      poseStamped.pose.position.y=poseGroundtruth(1);
      poseStamped.pose.orientation=geometryQuat;
      groundPath.poses.push_back(poseStamped);
      groundPathPub.publish(groundPath);

      //transfor groundtruth
      truthtransformStamped.header.stamp=currentTime;
      truthtransformStamped.transform.translation.x=poseGroundtruth(0);
      truthtransformStamped.transform.translation.y=poseGroundtruth(1);
      truthtransformStamped.transform.rotation=geometryQuat;
      br.sendTransform(truthtransformStamped);

      //transform slam
      q.setRPY(0,0,slamPose(2));
      geometryQuat.x=q.x();
      geometryQuat.y=q.y();
      geometryQuat.z=q.z();
      geometryQuat.w=q.w();
      transformStamped.header.stamp=currentTime;
      transformStamped.transform.translation.x=slamPose(0);
      transformStamped.transform.translation.y=slamPose(1);
      transformStamped.transform.rotation=geometryQuat;
      br.sendTransform(transformStamped);

      //path slam
      poseStamped.pose.position.x=slamPose(0);
      poseStamped.pose.position.y=slamPose(1);
      poseStamped.pose.orientation=geometryQuat;
      slamPath.poses.push_back(poseStamped);
      slamPathPub.publish(slamPath);

      // record the currentTime as lastTime
      lastTime=currentTime;
      // record the current command and next iteration, it will become the previous command.
      uk=u;
      // time pass
      ros::Duration(timeInterval).sleep();
    }
  }

    if (debug=="false")
  {

    // for transformation between Quaternion and RPY
    tf2::Quaternion q;
    geometry_msgs::Quaternion geometryQuat;
    tf2::Matrix3x3 m;
    
    // first detect whether the gazebo starts properly, which means ground truth can be obtained.
    // and then initialize poseGroundtruth.
    gazebo_msgs::GetModelState groundtruth;

    // detectRadius
    double detectRadius=0.8;
    ros::param::get("~detectRadius",detectRadius);

    while(ros::ok())
    {
      groundtruth.request.model_name=robotName;
      if (StatesClient.call(groundtruth))
      {
        poseGroundtruth(0)=groundtruth.response.pose.position.x;
        poseGroundtruth(1)=groundtruth.response.pose.position.y;
        q.setX(groundtruth.response.pose.orientation.x);
        q.setY(groundtruth.response.pose.orientation.y);
        q.setZ(groundtruth.response.pose.orientation.z);
        q.setW(groundtruth.response.pose.orientation.w);
        m.setRotation(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        poseGroundtruth(2)=yaw;
        // ROS_INFO("%f %f %f",poseGroundtruth(0),poseGroundtruth(1),poseGroundtruth(2));
        break;
      }else
      {
        ROS_INFO("fail to call /gazebo/get_model_state for nuturtle(groudtruth).");
      }
      ros::spinOnce();
    }

    // dt is a more accurate time interval between two iterations calculated by ros::Time.
    double dt=timeInterval;
    ros::Time currentTime,lastTime;
    currentTime=ros::Time::now();
    lastTime=ros::Time::now();

    // obTemp is (r,theta). consistent with laser data
    // it will be calculated by landmarks position in base_scan frame 
    Eigen::Vector2f obTemp;

    // k means iteration k;k+1 means now
    // so uk record the previous command input.
    Eigen::Vector3f uk;
    uk(0)=0;
    uk(1)=0;
    uk(2)=0;

    // slamPose is the pose calculated by ekf
    // initialize slamPose
    Eigen::Vector3f slamPose;
    slamPose(0)=0;
    slamPose(1)=0;
    slamPose(2)=0;

    // record the landmarks position in the world frame
    // record the detected landmarks position in the word frame
    std::vector<Eigen::Vector2f> globalLandmarks;
    std::vector<int> globalLandmarksCount;
    std::vector<Eigen::Vector2f> globalObservedLandmarks;
    Eigen::Vector2f obGlobalPose;

    // data association m distance
    Eigen::MatrixXf Rslam;
    Eigen::MatrixXf Hslam;
    Eigen::MatrixXf Pslam;
    Eigen::VectorXf xslam;
    Eigen::MatrixXf Qslam;
    Eigen::VectorXf z_predictslam;
    Eigen::VectorXf x_predictslam;

    // ekf
    while(ros::ok())
    { 
      // if the service call succeed, go ahead; otherwise, use the previous state.
      //get the current observation and record current time
      ros::spinOnce();
      currentTime=ros::Time::now();
      // calculate the actual time interval
      dt=(currentTime-lastTime).toSec();
      

      if (StatesClient.call(groundtruth))
      {
        poseGroundtruth(0)=groundtruth.response.pose.position.x;
        poseGroundtruth(1)=groundtruth.response.pose.position.y;
        q.setX(groundtruth.response.pose.orientation.x);
        q.setY(groundtruth.response.pose.orientation.y);
        q.setZ(groundtruth.response.pose.orientation.z);
        q.setW(groundtruth.response.pose.orientation.w);
        m.setRotation(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        poseGroundtruth(2)=yaw;
        // ROS_INFO("%f %f %f",poseGroundtruth(0),poseGroundtruth(1),poseGroundtruth(2));
      }
      
      // calculate the observations based on landmark positions in base_scan frame
      if (int(r->x.size())>0)
      {
        for (int i=0;i<int(r->x.size());i++)
        {
          double dx=0,dy=0;
          dx=r->x[i];
          dy=r->y[i];
          obTemp(0)=sqrt(pow(dx,2)+pow(dy,2));
          if (obTemp(0)<detectRadius)
          {
            obTemp(1)=atan2(dy,dx);
            observation.push_back(obTemp);
            obGlobalPose(0)=slamPose(0)+cos(slamPose(2))*dx-sin(slamPose(2))*dy;
            obGlobalPose(1)=slamPose(1)+sin(slamPose(2))*dx+cos(slamPose(2))*dy;
            globalObservedLandmarks.push_back(obGlobalPose);
          }
        }
      }



      // data association

      int data_association_method=1;
      ros::param::get("~data_association_method",data_association_method);

      if (data_association_method==1)
      {
        // if Hamming distance is less than associatedDis, should be same.
        // if Hamming distance is bigger than newDis, new landmark

        double associatedDis=0.05;
        double newDis=0.15;

        ros::param::get("~associatedDis",associatedDis);
        ros::param::get("~newDis",newDis);

        if (int(observation.size()>0))
        {
          // if there is no global landmarks, all counts.
          if (int(globalLandmarks.size())==0)
          {
            globalLandmarks=globalObservedLandmarks;
            for (int i=0;i<int(globalObservedLandmarks.size());i++)
            {
              observationIndex.push_back(i);
              globalLandmarksCount.push_back(1);
            }
          }else
          {
            // for each observed landmark
            for (int i=0;i<int(globalObservedLandmarks.size());i++)
            {
              // minimum H dis
              double mind;
              mind=10000;

              // minimum dis index
              int minIndex=0;

              // find the minimum dis2 and index
              for (int j=0;j<int(globalLandmarks.size());j++)
              {
                double tempd=0;
                tempd=fabs(globalLandmarks[j](0)-globalObservedLandmarks[i](0))+fabs(globalLandmarks[j](1)-globalObservedLandmarks[i](1));

                if (tempd<=mind)
                {
                  mind=tempd;
                  minIndex=j;
                }
              }
              if (mind<associatedDis)
              {
                // if associated
                observationIndex.push_back(minIndex);
                //average the position
                if (globalLandmarksCount[minIndex]>5)
                {
                  globalLandmarksCount[minIndex]=5;
                }
                globalLandmarks[minIndex](0)=(globalLandmarksCount[minIndex]*globalLandmarks[minIndex](0)+globalObservedLandmarks[i](0))/(globalLandmarksCount[minIndex]+1);
                globalLandmarks[minIndex](1)=(globalLandmarksCount[minIndex]*globalLandmarks[minIndex](1)+globalObservedLandmarks[i](1))/(globalLandmarksCount[minIndex]+1);
                globalLandmarksCount[minIndex]=globalLandmarksCount[minIndex]+1;
              }else
              {
                // if not associated
                if (mind>newDis)
                {
                  // if away enough
                  globalLandmarks.push_back(globalObservedLandmarks[i]);
                  observationIndex.push_back(int(globalLandmarks.size())-1);
                  globalLandmarksCount.push_back(1);
                }else
                {
                  // outliers
                  observationIndex.push_back(-1);
                }
              }
            }
          }
        }

        // delete outliers
        if (int(observationIndex.size())>0)
        {
          for (int i=int(observationIndex.size())-1;i>-1;i--)
          {
            if (observationIndex[i]==-1)
            {
              observationIndex.erase(observationIndex.begin()+i);
              observation.erase(observation.begin()+i);
            }
          }
        }

        // this part: if the landmark observed enough times, it counts.
        // BUT it does not make a big difference in the simulation.

        // if (int(observationIndex.size())>0)
        // {
        //   for (int i=int(observationIndex.size())-1;i>-1;i--)
        //   {
        //     if (globalLandmarksCount[observationIndex[i]]<10)
        //     {
        //       observationIndex.erase(observationIndex.begin()+i);
        //       observation.erase(observation.begin()+i);
        //     }
        //   }
        // }

      }
      

      if (data_association_method==2)
      {
        // use Mahalanobis distance
        // in pratical, it is hard to tune the threshold.

        // threshold
        // if corvarianceI==1, corvariance matrix is identity
        // if ==0, calculated by H,R,etc.
        // Calculated orvarianceI is very abnormal, it will cause Mahalanobis distance
        // too big, and too many new landmarks are included.

        double threshold=0.5;
        int corvarianceI=1;

        ros::param::get("~threshold",threshold);
        ros::param::get("~corvarianceI",corvarianceI);

        // number of landmarks
        int nofLandmarks=0;

        if (int(observation.size())>0)
        {
          xslam=ekfCal.returnx();
          if (int(xslam.size())==3)
          {
            for (int i=0;i<int(observation.size());i++)
            {
              observationIndex.push_back(i);
            }
          }else
          {
            // R,P,Q,H,z_predict,x_predict from ekf.
            Rslam=ekfCal.returnR();
            Pslam=ekfCal.returnP();
            Qslam=ekfCal.calQ(uk*dt);
            z_predictslam=ekfCal.returnz_predict(uk*dt);
            x_predictslam=ekfCal.predictx(uk*dt);
            Hslam=ekfCal.returnH(x_predictslam);
            Pslam.block<3,3>(0,0)=Pslam.block<3,3>(0,0)+Qslam;

            // corvariance matrix
            Eigen::MatrixXf corvarianceData;
            Eigen::MatrixXf IncorvarianceData;

            if (corvarianceI==1)
            {
              IncorvarianceData=Eigen::MatrixXf::Identity(int(z_predictslam.size()),int(z_predictslam.size()));
            }else
            {
              corvarianceData=Hslam*Pslam*Hslam.transpose()+Rslam;
              IncorvarianceData=corvarianceData.inverse();
            }
            
            // find the minimum distance 
            for (int i=0;i<int(observation.size());i++)
            {
              double minD;
              minD=10000;
              int minIndex;
              minIndex=0;
              nofLandmarks=int(z_predictslam.size())/2;
              for (int j=0;j<int(z_predictslam.size())/2;j++)
              {
                int f=2*j;
                Eigen::MatrixXf Md(1,1);
                double MdV=0;
                Eigen::MatrixXf dzj(2,1);
                dzj(0,0)=observation[i](0)-z_predictslam(f);
                dzj(1,0)=observation[i](1)-z_predictslam(f+1);
                Md=dzj.transpose()*IncorvarianceData.block(f,f,2,2)*dzj;
                MdV=Md(0,0);
                if (fabs(MdV)<minD)
                {
                  minD=fabs(MdV);
                  minIndex=j;
                }
              }
              //std::cout<<minD<<std::endl;
              if (minD<threshold)
              {
                observationIndex.push_back(minIndex);
              }else
              {
                std::cout<<nofLandmarks<<std::endl;
                std::cout<<minD<<std::endl;
                nofLandmarks++;
                observationIndex.push_back(nofLandmarks-1);
              }
            }
          }
        }
      }

      globalObservedLandmarks.clear();
      globalObservedLandmarks.shrink_to_fit();

      // call ekf to calculate the current pose based on previous command
      slamPose=ekfCal.ekfCall(observationIndex, observation,uk*dt);
      
      // free memory

      observationIndex.clear();
      observationIndex.shrink_to_fit();
      observation.clear();
      observation.shrink_to_fit();

      //path groundtruth
      geometryQuat.x=q.x();
      geometryQuat.y=q.y();
      geometryQuat.z=q.z();
      geometryQuat.w=q.w();
      poseStamped.header.stamp=currentTime;
      poseStamped.pose.position.x=poseGroundtruth(0);
      poseStamped.pose.position.y=poseGroundtruth(1);
      poseStamped.pose.orientation=geometryQuat;
      groundPath.poses.push_back(poseStamped);
      groundPathPub.publish(groundPath);

      //transfor groundtruth
      truthtransformStamped.header.stamp=currentTime;
      truthtransformStamped.transform.translation.x=poseGroundtruth(0);
      truthtransformStamped.transform.translation.y=poseGroundtruth(1);
      truthtransformStamped.transform.rotation=geometryQuat;
      br.sendTransform(truthtransformStamped);

      //transform slam
      q.setRPY(0,0,slamPose(2));
      geometryQuat.x=q.x();
      geometryQuat.y=q.y();
      geometryQuat.z=q.z();
      geometryQuat.w=q.w();
      transformStamped.header.stamp=currentTime;
      transformStamped.transform.translation.x=slamPose(0);
      transformStamped.transform.translation.y=slamPose(1);
      transformStamped.transform.rotation=geometryQuat;
      br.sendTransform(transformStamped);

      //path slam
      poseStamped.pose.position.x=slamPose(0);
      poseStamped.pose.position.y=slamPose(1);
      poseStamped.pose.orientation=geometryQuat;
      slamPath.poses.push_back(poseStamped);
      slamPathPub.publish(slamPath);

      
      // record the currentTime as lastTime
      lastTime=currentTime;
      // record the current command and next iteration, it will become the previous command.
      uk=u;
      // time pass
      
      ros::Duration(timeInterval).sleep();
    }
  }
  

  return 0;
  
}


