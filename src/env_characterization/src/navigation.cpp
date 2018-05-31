#include <ros/ros.h>
#include <bits/stdc++.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>

#include <env_characterization/PathNode.h>
#include <env_characterization/PathNodeArray.h>
#include <env_characterization/Action.h>
#include <env_characterization/action_srv.h>

using namespace std;

env_characterization::PathNodeArray path;

vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435};
vector<int> car_actions = {0, 0, 0, 0, 0, 0, 0, 0};
vector<int> snake_actions = {1, 2, 3, 4};
vector<vector<int>> action_fns = {car_actions, snake_actions};

string car_path = "/home/jonathan/.gazebo/models/smore_nest/smore_nest.sdf";
string snake_path = "/home/jonathan/.gazebo/models/snake_config/snake_config.sdf";
vector<string> config_path = {car_path, snake_path};

void yaw_nav(double goalYaw, tf::TransformListener *listener, ros::ServiceClient *act)
{
  double reachThresh = 0.01;
  double maxW = 2.0;
  ros::Rate rate(10);
  while (ros::ok())
  {
    try
    {
      tf::StampedTransform egoTF;
      listener->lookupTransform("map", "nest", ros::Time(), egoTF);
      
      double eYaw = goalYaw - tf::getYaw(egoTF.getRotation());
      
      if (eYaw <= reachThresh) break;
      
      double W = 20 * eYaw;
      
      double scaleFactor = min(1.0, maxW / abs(W));
      
      env_characterization::action_srv action;
      action.request.action.action = 0;
      action.request.action.vel.angular.z = W * scaleFactor;
      act->call(action);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("Couldn't get TF for yaw");
    }
    
    rate.sleep();
    ros::spinOnce();
  }
}

void climb_action(ros::ServiceClient *act)
{
  std_msgs::Float32 height;
  height.data = 1.0;  
  
  env_characterization::action_srv action;
  action.request.action.action = 3;
  act->call(action);
}

void climb_rev_action(ros::ServiceClient *act)
{
  std_msgs::Float32 height;
  height.data = 1.0;
  
  env_characterization::action_srv action;
  action.request.action.action = 4;
  act->call(action);
}

void straight_nav(const tf::Transform &goalTF, tf::TransformListener *listener,
	       tf::TransformBroadcaster *br, ros::ServiceClient *act)
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
      
      double eX = egoTF.getOrigin().getX();
      
      //cout << "Errors: " << eX << ", " << eY << endl;
      
      if (abs(eX) <= reachThresh) break;
      
      double V = 10 * eX;
      
      //cout << "V, W: " << V << ", " << W << endl;
      
      double scaleFactor = min(1.0, maxVel / abs(V));
      
      V *= scaleFactor;
      
      //cout << "Scaled V, W: " << V << ", " << W << endl;
      
      env_characterization::action_srv action;
      action.request.action.action = 0;
      action.request.action.vel.linear.x = V;
      act->call(action);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Transform to ego frame not found");
    }
    rate.sleep();
    ros::spinOnce();
  }
}

void waypt_nav(const tf::Transform &goalTF, tf::TransformListener *listener,
	       tf::TransformBroadcaster *br, ros::ServiceClient *act)
{
  double reachThresh = 0.05;
  double maxVel = 1.0;
  
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
      
      if (sqrt(pow(eX, 2) + pow(eY, 2)) <= reachThresh) break;
      
      double V = 10 * eX;
      double W = 200 * eY;
      
      //cout << "V, W: " << V << ", " << W << endl;
      
      double scaleFactor = min(1.0, maxVel / max(abs(V), abs(W)));
      
      V *= scaleFactor;
      W *= scaleFactor;
      
      //cout << "Scaled V, W: " << V << ", " << W << endl;
      
      env_characterization::action_srv action;
      action.request.action.action = 0;
      action.request.action.vel.linear.x = V;
      action.request.action.vel.angular.z = W;
      act->call(action);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Transform to ego frame not found");
    }
    rate.sleep();
    ros::spinOnce();
  }  
}

void path_cb(env_characterization::PathNodeArrayConstPtr msg)
{
  path = *msg;
}

void spawn_config(string config_path, geometry_msgs::Pose p, ros::ServiceClient *spawn)
{
  std::ifstream in(config_path);
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string test = buffer.str();
  std::cout << test << std::endl << std::endl;

  gazebo_msgs::SpawnModel s1;
  s1.request.model_name = "nest";
  s1.request.model_xml = test;
  s1.request.robot_namespace = "nest";
  s1.request.initial_pose = p;
  s1.request.reference_frame = "map";

  if (spawn->call(s1))
  {
    ROS_INFO("success");
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle node;
  ros::Rate rate(5);
  
  ros::Publisher cmd_pub = node.advertise<env_characterization::Action>("/cmd_action", 5);
  
  ros::Subscriber path_sub = node.subscribe("/path", 1, path_cb);
  
  tf::TransformListener listener;
  tf::TransformBroadcaster br;
  
  ros::ServiceClient spawn = node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  ros::ServiceClient del = node.serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
  ros::ServiceClient act = node.serviceClient<env_characterization::action_srv> ("/cmd_action");
  
  string filename = "/home/jonathan/.gazebo/models/smore_nest/smore_nest.sdf";
  
  geometry_msgs::Pose pose_init;
  pose_init.position.x = 0.0;
  pose_init.position.y = 0.0;
  pose_init.position.z = 0.0;
  pose_init.orientation = tf::createQuaternionMsgFromYaw(0.0);
  
  //spawn_config(filename, pose_init, &spawn);
  
  //ros::Duration(3.0).sleep();
  
  while (path.path.size() == 0 && ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }  
  
  //Follow Path
  env_characterization::PathNodeArray cur_path = path;
  int curConfig = cur_path.path[0].config;
  for (int i = 0; i < cur_path.path.size(); i++)
  {
    env_characterization::PathNode node = cur_path.path[i];
    geometry_msgs::Pose pose = node.pose;
    
    //Check if need to reconfigure
    if (curConfig != node.config)
    {
      curConfig = node.config;
      //Reorient robot
      ROS_INFO("Reorienting robot");
      yaw_nav(node.theta, &listener, &act);
      ROS_INFO("Reoriented robot");
      
      //Stop Robot
      env_characterization::action_srv action;
      action.request.action.action = 0;
      action.request.action.vel.linear.x = 0;
      action.request.action.vel.angular.z = 0;
      act.call(action);
      
      ros::Duration(1).sleep();
      
      //Record current orientation
      tf::StampedTransform egoTF;
      while (ros::ok())
      {
	try
	{
	  ros::spinOnce();
	  listener.lookupTransform("map", "nest", ros::Time(), egoTF);
	  break;
	}
	catch(tf::TransformException& ex)
	{
	  ROS_ERROR("Transform to ego frame not found");
	}
	rate.sleep();
      }
      
      geometry_msgs::Pose config_pose;
      config_pose.position.x = egoTF.getOrigin().getX();
      config_pose.position.y = egoTF.getOrigin().getY();
      config_pose.position.z = egoTF.getOrigin().getZ();
      
      config_pose.orientation.x = egoTF.getRotation().getX();
      config_pose.orientation.y = egoTF.getRotation().getY();
      config_pose.orientation.z = egoTF.getRotation().getZ();
      config_pose.orientation.w = egoTF.getRotation().getW();    
      
      //Delete old config
      ROS_INFO("Deleting old config");
      
      gazebo_msgs::DeleteModel del_req;
      del_req.request.model_name = "nest";
      if (del.call(del_req))
      {
	ROS_INFO("success");
      }
      else
      {
	ROS_ERROR("Failed to call service");
	return 1;
      }
      ROS_INFO("Deleted old config");
      
      ros::Duration(1).sleep();
      
      //Spawn desired config
      ROS_INFO("Spawning new config");
      
      spawn_config(config_path[node.config], config_pose, &spawn);
      
      ROS_INFO("Reconfigured");
      
      ros::Duration(3).sleep();
    }
    
    tf::Transform goalTF;
    goalTF.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0));
    goalTF.setRotation(tf::createQuaternionFromYaw(0.0));
    
    cout << "Begining Nav to Pt " << i << ": " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << endl;
    
    int action_fn = action_fns[node.config][node.action];
    
    cout << "Action: " << action_fn << endl;
    
    switch (action_fn)
    {
      case 0:
	waypt_nav(goalTF, &listener, &br, &act);
	break;
      case 1:
	straight_nav(goalTF, &listener, &br, &act);
	break;
      case 2:
	straight_nav(goalTF, &listener, &br, &act);
      case 3:
	climb_action(&act);
	break;
      case 4:
	climb_rev_action(&act);
	break;
    }
    
    cout << "Finished Pt " << i << endl;
    
    ros::spinOnce();
    rate.sleep();
  }
  
  //Stop Robot
  env_characterization::action_srv action;
  action.request.action.vel.linear.x = 0;
  action.request.action.vel.angular.z = 0;
  act.call(action);
  
  cout << "Completed Path" << endl;
}