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
///     landmarks (nuslam::TurtleMap): the centers and radius of detected landmarks
/// SUBSCRIBES:
///     scan (sensor_msgs::LaserScan): laser scan data



#include <iosfwd> 
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include "geometryfitting/geometryfitting.hpp"
#include <sensor_msgs/LaserScan.h>
#include <nuslam/TurtleMap.h>
#include <rigid2d/rigid2d.hpp>
#include <Eigen/Dense>


static sensor_msgs::LaserScan::ConstPtr r=nullptr;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    r=msg;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "landmarks");

    ros::NodeHandle n;
    
    double timeInterval=0.01;
    ros::param::get("timeInterval",timeInterval);

    ros::Subscriber laserSub=n.subscribe("/scan",1,laserCallback);
    ros::Publisher landmarksPub=n.advertise<nuslam::TurtleMap>("landmarks",1);
    double lenCluster=0, radiusCluster=0.05;
    int sizeCluster=0;
    lenCluster=0.1;
    sizeCluster=4;

    std::vector<std::vector<std::vector<double>>> clusters;

    double angleIncre=0;
    double angleMin=0;
    while (ros::ok())
    {
        if (r!=nullptr)
        {
            angleMin=r->angle_min;
            double angle=angleMin;
            int n=r->ranges.size();
            angleIncre=2.0*rigid2d::PI/n;
            std::vector<std::vector<double>> pointSet;
            std::vector<double> point;
            point.push_back(r->ranges[0]*cos(angle));
            point.push_back(r->ranges[0]*sin(angle));
            pointSet.push_back(point);
            for (int i=1;i<n;i++)
            {
                angle+=angleIncre;
                if (fabs(r->ranges[i]-r->ranges[i-1])<lenCluster)
                {
                    point[0]=r->ranges[i]*cos(angle);
                    point[1]=r->ranges[i]*sin(angle);
                    pointSet.push_back(point);
                }else
                {
                    if (int(pointSet.size())>sizeCluster)
                    {
                        clusters.push_back(pointSet);
                    }
                    pointSet.clear();
                }
            }
            for (int i=0;i<n;i++)
            {
                angle+=angleIncre;
                if (i==0)
                {
                    if (fabs(r->ranges[i]-r->ranges[int(r->ranges.size()-1)])<lenCluster)
                    {
                        point[0]=r->ranges[i]*cos(angle);
                        point[1]=r->ranges[i]*sin(angle);
                        pointSet.push_back(point);
                    }else
                    {
                        if (int(pointSet.size())>sizeCluster)
                        {
                            clusters.push_back(pointSet);
                        }
                        pointSet.clear();
                        break;
                    }
                }else
                {
                    if (fabs(r->ranges[i]-r->ranges[i-1])<lenCluster)
                    {
                        point[0]=r->ranges[i]*cos(angle);
                        point[1]=r->ranges[i]*sin(angle);
                        pointSet.push_back(point);
                    }else
                    {
                        if (int(pointSet.size())>sizeCluster)
                        {
                            clusters.push_back(pointSet);
                        }
                        pointSet.clear();
                        break;
                    }
                }
            }
            if ((int)clusters.size()>0)
            {
                if(fabs(r->ranges[0]*cos(angleMin)-clusters[0][0][0])+fabs(r->ranges[0]*sin(angleMin)-clusters[0][0][1])<0.001)
                {
                    clusters.erase(clusters.begin());
                }
            }
            std::vector<float> landmx;
            std::vector<float> landmy;
            std::vector<float> landmr;
            for (int i=int(clusters.size())-1;i>-1;i--)
            {
                int nofc=clusters[i].size();
                Eigen::VectorXf pointx(nofc);
                Eigen::VectorXf pointy(nofc);
                for (int j=0;j<nofc;j++)
                {
                    pointx(j)=clusters[i][j][0];
                    pointy(j)=clusters[i][j][1];
                }
                Eigen::VectorXf result;
                result=geometryfitting::circlefitting(pointx,pointy);
                if (result(2)>radiusCluster)
                {
                    clusters.erase(clusters.begin()+i);
                }else
                {
                    landmx.push_back(result(0));
                    landmy.push_back(result(1));
                    landmr.push_back(result(2));
                    // ROS_INFO("%f %f %f %f",result(0),result(1),result(2),result(3));
                }
                
            }

            // ROS_INFO("%d",clusters.size());

            nuslam::TurtleMap landmarksData;
            landmarksData.x=landmx;
            landmarksData.y=landmy;
            landmarksData.radius=landmr;

            landmarksPub.publish(landmarksData);

            landmx.clear();
            landmx.shrink_to_fit();
            landmy.clear();
            landmy.shrink_to_fit();
            landmr.clear();
            landmr.shrink_to_fit();
            point.clear();
            point.shrink_to_fit();
            pointSet.clear();
            pointSet.shrink_to_fit();
            clusters.clear();
            clusters.shrink_to_fit();
            r=nullptr;
        }else
        {

        }
        ros::Duration(timeInterval).sleep();
        ros::spinOnce();
    }
    

    
    return 0;
  
}