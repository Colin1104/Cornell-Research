#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <bits/stdc++.h>
#include <geometry_msgs/Pose.h>

using namespace std;

gazebo_msgs::ModelStates states;

void states_cb(const gazebo_msgs::ModelStatesConstPtr msg)
{
  states = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gazebo_tf_node");
  ros::NodeHandle node;
  ros::Rate rate(10);
    
  ros::Subscriber sub = node.subscribe("/gazebo/model_states", 1, states_cb);
  
  tf::TransformBroadcaster br;
  
  while (states.name.size() == 0)
  {
    ros::spinOnce();
  }
  
  ROS_INFO("Got Message");
  
  while (ros::ok())
  {
    for (int i = 0; i < states.name.size(); i++)
    {
      if (states.name[i] == "ground") continue;
      
      geometry_msgs::Pose pose = states.pose[i];
      tf::Transform transform;
      transform.setRotation(tf::Quaternion(pose.orientation.x,
					  pose.orientation.y,
					  pose.orientation.z,
					  pose.orientation.w));
      transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
      
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", states.name[i]));
    }
    
    rate.sleep();
    ros::spinOnce();
  }
  
  return 0;
}