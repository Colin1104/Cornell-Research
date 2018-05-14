#include <ros/ros.h>
#include <bits/stdc++.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <bits/stdc++.h>

#include <gazebo_msgs/SpawnModel.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

using namespace std;

nav_msgs::Path path;

void waypt_nav(const tf::Transform &goalTF, tf::TransformListener *listener,
	       tf::TransformBroadcaster *br, ros::Publisher *cmd_pub)
{
  double reachThresh = 0.05;
  double maxVel = 3.0;
  
  ros::Rate rate(5);
  
  while (ros::ok())
  {
    try
    {
      tf::StampedTransform egoTF;
      br->sendTransform(tf::StampedTransform(goalTF, ros::Time::now(), "map", "goal"));
      ros::spinOnce();
      listener->lookupTransform("nest", "goal", ros::Time(), egoTF);
      
      double theta = tf::getYaw(egoTF.getRotation());
      
      double eX = egoTF.getOrigin().getX();
      double eY = egoTF.getOrigin().getY();
      
      //cout << "Errors: " << eX << ", " << eY << endl;
      
      if (sqrt(pow(eX, 2) + pow(eY, 2)) < reachThresh) break;
      
      double V = 10 * eX;
      double W = 500 * eY;
      
      //cout << "V, W: " << V << ", " << W << endl;
      
      double scaleFactor = min(1.0, maxVel / max(abs(V), abs(W)));
      
      V *= scaleFactor;
      W *= scaleFactor;
      
      //cout << "Scaled V, W: " << V << ", " << W << endl;
      
      geometry_msgs::Twist cmd;
      cmd.linear.x = V;
      cmd.angular.z = W;
      
      cmd_pub->publish(cmd);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Transform to ego frame not found");
    }
    rate.sleep();
    ros::spinOnce();
  }  
}

void path_cb(nav_msgs::PathConstPtr msg)
{
  path = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle node;
  ros::Rate rate(5);
  
  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/turtlebot_teleop/cmd_vel", 5);
  
  ros::Subscriber path_sub = node.subscribe("/path", 1, path_cb);
  
  tf::TransformListener listener;
  tf::TransformBroadcaster br;
  
  string filename = "/home/jonathan/.gazebo/models/smore_nest/smore_nest.sdf";
  
  std::ifstream in(filename);
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string test = buffer.str();
  std::cout << test << std::endl << std::endl;
  
  ros::ServiceClient spawn = node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");

  gazebo_msgs::SpawnModel s1;
  geometry_msgs::Pose p1;
  s1.request.model_name = "nest";
  s1.request.model_xml = test;
  s1.request.robot_namespace = "nest";
  p1.position.x = 0.0;
  p1.position.y = 0.0;
  p1.position.z = 0.0;
  p1.orientation = tf::createQuaternionMsgFromYaw(1.5708);
  s1.request.initial_pose = p1;
  s1.request.reference_frame = "";

  if (spawn.call(s1))
  {
    ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
    return 1;
  }
  
  sleep(5);
  
  while (path.poses.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  
  //Follow Path
  nav_msgs::Path cur_path = path;  
  for (int i = cur_path.poses.size() - 1; i >= 0; i--)
  {
    geometry_msgs::Pose pose = cur_path.poses[i].pose;
    
    tf::Transform goalTF;
    goalTF.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    goalTF.setRotation(tf::createQuaternionFromYaw(0.0));
    
    cout << "Begining Nav to Pt " << i << ": " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << endl;
    waypt_nav(goalTF, &listener, &br, &cmd_pub);
    cout << "Finished Pt " << i << endl;
    
    ros::spinOnce();
    rate.sleep();
  }
  
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.angular.z = 0;
  
  cmd_pub.publish(cmd);
  cout << "Completed Path" << endl;
}