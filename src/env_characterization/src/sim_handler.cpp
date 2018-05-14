#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include<iostream>
#include<fstream>
#include <tf/transform_datatypes.h>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_handler");
  ros::NodeHandle node;
  
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
  p1.orientation = tf::createQuaternionMsgFromYaw(0.0);
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
   
   sleep(2);
   
   double xTarg = 1.0;
  double yTarg = 1.0;
}