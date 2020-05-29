/// \file
/// \brief draw_map based on the topic landmarks and topic MapIndexVisual to draw the landmarks
///        in rviz. MapIndexVisual is to determine which landmarks to be visual. If it is null,
///        all the landmarks are visual. It is used to simulate the detected raidus in dubug model.
///
/// PARAMETERS:
///     private:
///     frame_id (str): landmarks position in frame_id frame
///     timeInterval (double): the time duration of the markers
/// PUBLISHES:
///     visualization_marker_array (visualization_msgs::MarkerArray): marker array to visualize the landmarks in rviz
/// SUBSCRIBES:
///     landmarks (nuslam::TurtleMap): the centers and radius of detected landmarks
///     MapIndexVisual (visualization_msgs::MarkerArray): the index of landmarks to be visual



#include <iosfwd> 
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include <nuslam/TurtleMap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nuslam/MapIndexVisual.h>

static ros::Publisher markerPub;
static visualization_msgs::Marker marker;
static visualization_msgs::MarkerArray landmarks;
static nuslam::MapIndexVisual::ConstPtr r=nullptr;

void landmarksCallback(const nuslam::TurtleMap::ConstPtr &msg)
{
    marker.header.stamp = ros::Time::now();
    if (r!=nullptr)
    {
        if (int(r->index.size())>0)
        {
            for (int i=0;i<int(r->index.size());i++)
            {
                marker.id+=1;
                marker.pose.position.x=msg->x[r->index[i]];
                marker.pose.position.y=msg->y[r->index[i]];
                // marker.scale.x = msg->radius[i];
                // marker.scale.y = msg->radius[i];
                marker.scale.x = 0.07;
                marker.scale.y = 0.07;
                landmarks.markers.push_back(marker);
            }
            marker.id=0;
            markerPub.publish(landmarks);
            landmarks.markers.clear();
            landmarks.markers.shrink_to_fit();
            return;
        }
    }

    // if MapIndexVisual topic has no message
    // all the landmarks are visual
    for (int i=0;i<int(msg->radius.size());i++)
    {
        marker.id+=1;
        marker.pose.position.x=msg->x[i];
        marker.pose.position.y=msg->y[i];
        // marker.scale.x = msg->radius[i];
        // marker.scale.y = msg->radius[i];
        marker.scale.x = 0.07;
        marker.scale.y = 0.07;
        landmarks.markers.push_back(marker);
    }
    marker.id=0;
    markerPub.publish(landmarks);
    landmarks.markers.clear();
    landmarks.markers.shrink_to_fit();
    


}

void indexCallback(const nuslam::MapIndexVisual::ConstPtr &msg)
{
    r=msg;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "draw_map");

    ros::NodeHandle n;

    // timeInterval: the duration of the markers in rviz
    double timeInterval=0.1;
    ros::param::get("~timeInterval",timeInterval);

    // initial the marker
    std::string frame_id;
    ros::param::get("~frame_id", frame_id);
    marker.header.frame_id = frame_id;
    marker.ns = "landmarks";
    marker.id = 0;
    marker.type=visualization_msgs::Marker::CYLINDER;
    marker.action=visualization_msgs::Marker::ADD;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.scale.z = 0.1;
    marker.pose.position.z=0;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    marker.pose.orientation.w=1;
    marker.lifetime = ros::Duration(timeInterval);

    // subscribe the topics needed
    ros::Subscriber landmarksSub=n.subscribe("/landmarks",1,landmarksCallback);
    ros::Subscriber indexSub=n.subscribe("/MapIndexVisual",1,indexCallback);

    // publish the topics
    markerPub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
   
    ros::spin();
    return 0;
  
}