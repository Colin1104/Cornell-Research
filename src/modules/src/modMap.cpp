#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>
#include <octomap/math/Utils.h>
#include <cmath>

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <octomap_ros/conversions.h>

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace octomap_msgs;

// Globals:
tf::Transform dockOffset;
tf::Transform modOffset;
string dockTagString;
tf::Quaternion q;



string parse(string input, int id)
// String parsing function to extract data:
{
    // 0 = egoTagString, 1 = ego face, 2 = dockTagString, 3 = dock face
    std::string s = input;
    std::string delimiter = ":";

    size_t pos = 0;
    std::string token;
    int count = 0;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        if (count == id) {
            return token;
        }
        s.erase(0, pos + delimiter.length());
        count++;
    }
    return s;
}

void updateModOffset(string dataString)
// updates global modOffset variable based on dataString
{
  q.setRPY(0, 0, 0);
  modOffset.setRotation(q);
  modOffset.setOrigin( tf::Vector3(0.0, 0.16, 0.0) );
}

void updateDockOffset(string dataString)
// updates global dockOffset variable based on dataString
{
  q.setRPY(0, 0, 0);
  dockOffset.setRotation(q);
  dockOffset.setOrigin( tf::Vector3(0.0, 0.06, 0.0) );
}

void tag_info_cb(std_msgs::String dataString)
// Callback for tag information topic
{
  dockTagString = parse(dataString.data, 2); // TODO write me
  updateModOffset(dataString.data); // TODO write me
  updateDockOffset(dataString.data); // TODO write me
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "modMap");
  
  ros::NodeHandle node;
  
  ros::Rate rate(10.0);
  
  
  tf::Transform modTransform;
  tf::TransformListener listener;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::StampedTransform stransform;
  

  // Subscribe to the tag info topic:
  ros::Subscriber tagInfoSub = node.subscribe("/reconf_request", 10, tag_info_cb);
  
  sleep(5);
  
  try{
    ros::Time gTime = ros::Time::now();
    listener.waitForTransform("camera_rgb_frame", "tag_5", gTime, ros::Duration(15.0));
    listener.lookupTransform("camera_rgb_frame", "tag_5", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d origin(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  
  cout << origin.x() << " : " << origin.y() << " : " << origin.z() << endl;
  
  modTransform.setOrigin(stransform.getOrigin());
  modTransform.setRotation(stransform.getRotation());
  
  q.setRPY(0, 0, 0);
  modOffset.setRotation(q);
  modOffset.setOrigin( tf::Vector3(0.0, 0.16, 0.0) );
  
  q.setRPY(0, 0, 0);
  dockOffset.setRotation(q);
  dockOffset.setOrigin( tf::Vector3(0.0, 0.06, 0.0) );
  
  cout << "Transforms Saved" << endl;
  
  while (ros::ok())
  {
    br.sendTransform(tf::StampedTransform(modTransform, ros::Time::now(), "camera_rgb_frame", "map"));
    rate.sleep();
    /*
    br.sendTransform(tf::StampedTransform(modOffset, ros::Time::now(), "tag_4", "goal"));
    br.sendTransform(tf::StampedTransform(dockOffset, ros::Time::now(), "tag_4", "dock"));
    */
    br.sendTransform(tf::StampedTransform(modOffset, ros::Time::now(), dockTagString, "goal"));
    br.sendTransform(tf::StampedTransform(dockOffset, ros::Time::now(), dockTagString, "dock"));
    ros::spinOnce();
  }
}
