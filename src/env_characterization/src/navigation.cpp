#include <ros/ros.h>
#include <bits/stdc++.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <vector>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <env_characterization/PathNode.h>
#include <env_characterization/PathNodeArray.h>
#include <env_characterization/Action.h>
#include <env_characterization/action_srv.h>
#include <env_characterization/path_srv.h>

using namespace std;

env_characterization::PathNodeArray path;
tf::TransformListener* listener;
ros::ServiceClient path_client;
geometry_msgs::Pose goal;
bool pathActive = false;
int curConfig = 0;

vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435};
vector<int> car_actions = {0, 0, 0, 0, 0, 0, 0, 0};
vector<int> snake_actions = {1, 2, 3};
vector<int> dolphin_actions = {1, 4};
vector<vector<int>> action_fns = {car_actions, snake_actions, dolphin_actions};

string car_path = "/home/jonathan/.gazebo/models/smore_nest/smore_nest.sdf";
string snake_path = "/home/jonathan/.gazebo/models/snake_config/snake_config.sdf";
string dolphin_path = "/home/jonathan/.gazebo/models/dolphin_config/model.sdf";
vector<string> config_path = {car_path, snake_path, dolphin_path};

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
      
      double curYaw = tf::getYaw(egoTF.getRotation());
      if (curYaw < 0) curYaw += 2 * M_PI;
      double eYaw = goalYaw - curYaw;
      if (abs(goalYaw - curYaw) > 360 - abs(goalYaw - curYaw))
      {
	eYaw = goalYaw + 360 - curYaw;
      }
      
      cout << goalYaw << ", " << eYaw << endl;
      
      if (abs(eYaw) <= reachThresh) break;
      
      double W = 10 * eYaw;
      
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

void wave_nav(const tf::Transform &goalTF, tf::TransformListener *listener,
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
      listener->lookupTransform("map", "nest", ros::Time(), egoTF);
      tf::Transform projTF(tf::createQuaternionFromYaw(tf::getYaw(egoTF.getRotation())), egoTF.getOrigin());
      br->sendTransform(tf::StampedTransform(projTF, ros::Time::now(), "map", "nestProj"));
      br->sendTransform(tf::StampedTransform(goalTF, ros::Time::now(), "map", "goal"));
      ros::spinOnce();
      
      listener->lookupTransform("nestProj", "goal", ros::Time(), egoTF);
      
      double eX = egoTF.getOrigin().getX();
      
      //cout << "Errors: " << eX << ", " << eY << endl;
      
      if (abs(eX) <= reachThresh) break;
      
      double V = 10 * eX;
      
      //cout << "V, W: " << V << ", " << W << endl;
      
      double scaleFactor = min(1.0, maxVel / abs(V));
      
      V *= scaleFactor;
      
      //cout << "Scaled V, W: " << V << ", " << W << endl;
      
      env_characterization::action_srv action;
      action.request.action.action = 1;
      action.request.action.vel.linear.x = V / abs(V);
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
      listener->lookupTransform("map", "nest", ros::Time(), egoTF);
      tf::Transform projTF(tf::createQuaternionFromYaw(tf::getYaw(egoTF.getRotation())), egoTF.getOrigin());
      br->sendTransform(tf::StampedTransform(projTF, ros::Time::now(), "map", "nestProj"));
      br->sendTransform(tf::StampedTransform(goalTF, ros::Time::now(), "map", "goal"));
      ros::spinOnce();
      
      listener->lookupTransform("nestProj", "goal", ros::Time(), egoTF);
      
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
  
  ros::Rate rate(10);
  
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
      
      cout << "Scaled V, W: " << V << ", " << W << endl;
      
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

bool getPath(geometry_msgs::Pose goalPose)
{
  try
  {
    tf::StampedTransform egoTF;
    listener->lookupTransform("map", "nest", ros::Time(), egoTF);
    
    geometry_msgs::Pose startPose;
    startPose.position.x = egoTF.getOrigin().getX();
    startPose.position.y = egoTF.getOrigin().getY();
    
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(egoTF.getRotation(), quat);
    startPose.orientation = quat;
    
    float startYaw = tf::getYaw(egoTF.getRotation());
    
    env_characterization::path_srv pathSrv;
    pathSrv.request.start.pose = startPose;
    pathSrv.request.start.config = curConfig;
    pathSrv.request.start.theta = startYaw;
    
    cout << "Start Config, Theta: " << curConfig << ", " << startYaw << endl;
    
    pathSrv.request.goal.pose = goalPose;
    if (path_client.call(pathSrv))
    {
      path = pathSrv.response.path;
      return true;
    }
    else
    {
      cout << "Failed to get path" << endl;
      return false;
    }
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("Couldn't get TF for yaw");
  }
}

void goal_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
  cout << "Nav Goal Received" << endl;
  goal = msg->pose;
  
  if (getPath(goal))
  {
    pathActive = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_node");
  ros::NodeHandle node;
  ros::Rate rate(5);
  
  ros::Publisher cmd_pub = node.advertise<env_characterization::Action>("/cmd_action", 5);
  ros::Subscriber goal_sub = node.subscribe("/move_base_simple/goal", 1, goal_cb);
  //ros::Subscriber path_sub = node.subscribe("/path", 1, path_cb);
  
  listener = new tf::TransformListener;
  
  tf::TransformBroadcaster br;
  
  ros::ServiceClient spawn = node.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_sdf_model");
  ros::ServiceClient del = node.serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
  ros::ServiceClient act = node.serviceClient<env_characterization::action_srv> ("/cmd_action");
  path_client = node.serviceClient<env_characterization::path_srv>("/get_path");
  
  string filename = "/home/jonathan/.gazebo/models/smore_nest/smore_nest.sdf";
  
  geometry_msgs::Pose pose_init;
  pose_init.position.x = 0.5;
  pose_init.position.y = -0.5;
  pose_init.position.z = 0.0;
  pose_init.orientation = tf::createQuaternionMsgFromYaw(0.0);
  
  spawn_config(filename, pose_init, &spawn);
  
  //ros::Duration(3.0).sleep();
  
  int nWaypt = 0;
  
  path.path.clear();
  while (ros::ok())
  {
    if (path.path.size() > 0)
    {
      cout << "Path size: " << path.path.size() << endl;
      env_characterization::PathNode node = path.path.back();
      path.path.pop_back();
      cout << path.path.size() << endl;
      geometry_msgs::Pose pose = node.pose;
      
      //Check if need to reconfigure
      if (node.action == -1)
      {
	//Reorient robot
	if (curConfig == 0)
	{
	  ROS_INFO("Reorienting robot");
	  yaw_nav(node.theta, listener, &act);
	  ROS_INFO("Reoriented robot");
	}
	
	curConfig = node.config;
	
	//Stop Robot
	env_characterization::action_srv action;
	action.request.action.action = 0;
	action.request.action.vel.linear.x = 0;
	action.request.action.vel.angular.z = 0;
	act.call(action);
	
	ros::Duration(3).sleep();
	
	//Record current orientation
	tf::StampedTransform egoTF;
	while (ros::ok())
	{
	  try
	  {
	    ros::spinOnce();
	    listener->lookupTransform("map", "nest", ros::Time(), egoTF);
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
	
	if (node.config != 0) egoTF.setRotation(tf::createQuaternionFromYaw(node.theta));
	
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
	
	ros::Duration(6).sleep();
      }
      else
      {
	tf::Transform goalTF;
	goalTF.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0));
	goalTF.setRotation(tf::createQuaternionFromYaw(0.0));
	
	tf::StampedTransform egoTF;
	br.sendTransform(tf::StampedTransform(goalTF, ros::Time::now(), "map", "goal"));
	ros::spinOnce();
	while (ros::ok())
	{
	  try
	  {
	    ros::spinOnce();
	    listener->lookupTransform("nest", "goal", ros::Time(), egoTF);
	    break;
	  }
	  catch(tf::TransformException& ex)
	  {
	    ROS_ERROR("Transform to ego frame not found");
	  }
	  rate.sleep();
	}
	
	if (egoTF.getOrigin().length() > 0.15)
	{
	  cout << "Recalculating Path" << endl;
	  getPath(goal);
	}
	
	cout << "Begining Nav to Pt " << nWaypt << ": " << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << endl;
	
	cout << "Config, Action: " << node.config << ", " << node.action << endl;
	
	int action_fn = action_fns[node.config][node.action];
	
	cout << "Action: " << action_fn << endl;
	
	switch (action_fn)
	{
	  case 0:
	    waypt_nav(goalTF, listener, &br, &act);
	    break;
	  case 1:
	    straight_nav(goalTF, listener, &br, &act);
	    break;
	  case 2:
	    climb_action(&act);
	    break;
	  case 3:
	    climb_rev_action(&act);
	    break;
	  case 4:
	    wave_nav(goalTF, listener, &br, &act);
	    break;
	}
	
	cout << "Finished Pt " << nWaypt << endl;
	nWaypt++;
      }
    }
    else if (pathActive)
    {
      //Stop Robot
      env_characterization::action_srv action;
      action.request.action.vel.linear.x = 0;
      action.request.action.vel.angular.z = 0;
      act.call(action);
      
      pathActive = false;
      nWaypt = 0;
      
      cout << "Completed Path" << endl;
    }
    ros::spinOnce();
    rate.sleep();
  }
  
  //Follow Path
  /*env_characterization::PathNodeArray cur_path = path;
  int curConfig = cur_path.path[0].config;
  for (int i = 0; i < cur_path.path.size(); i++)
  {
    
  }*/
}