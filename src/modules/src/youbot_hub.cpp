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

#include <sensorCell.h>
#include <MinHeap.h>
#include <InfoNode.h>

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
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <octomap_ros/conversions.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "ik_solver_service/SolvePreferredPitchIK.h"
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace octomap_msgs;

bool mapReceived;
Octomap mapMsg;
double res = 0.02;
double rFactor = 1 / res;
tf::Quaternion q;
move_base_msgs::MoveBaseGoal goal;
bool nbvReceived = false;
bool goalRunning = false;
double camHeight;
double camPitch;
bool stationaryNBV;

ros::Publisher arm_pub_pos;

void nbv_cb(const std_msgs::Float64MultiArrayConstPtr& nbv)
{
  ROS_INFO("Navigation Hub has received NBV");
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  camHeight = nbv->data[2];
  camPitch = nbv->data[4];
  
  double x = nbv->data[0];
  double y = nbv->data[1];
  double width = pow((camHeight - 0.03) / (0.44 - 0.03),2) * (0.23 - 0.39) + 0.39;
  double theta = nbv->data[3];
  
  if (nbv->data.size() > 5)
  {
    stationaryNBV = true;
  }
  else
  {
    stationaryNBV = false;
    x += (-0.07 * cos(theta) + width * sin(theta));
    y += (-0.07 * sin(theta) - width * cos(theta));
  }
  
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  
  q.setRPY(0, 0, nbv->data[3]);
  goal.target_pose.pose.orientation.x = double(q.x());
  goal.target_pose.pose.orientation.y = double(q.y());
  goal.target_pose.pose.orientation.z = double(q.z());
  goal.target_pose.pose.orientation.w = double(q.w());
  
  nbvReceived = true;
}

void moveArmNavigate()
{
  brics_actuator::JointPositions command;
  vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(5);
  
  double cmdPoint[5] = { 2.949606 + 1.5707, 2.252030, -3.697981, 0.250400, 2.312561 };
  
  std::stringstream jointName;
  
  for (int i = 0; i < 5; ++i) {
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    
    armJointPositions[i].joint_uri = jointName.str();
    
    armJointPositions[i].value = cmdPoint[i];
    
    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
  
  command.positions = armJointPositions;
  
  arm_pub_pos.publish(command);
  sleep(0.1);
  arm_pub_pos.publish(command);
  
  return;
}

void moveArm(ros::NodeHandle *nodePtr, double height, double pitch, double yaw)
{
  ros::ServiceClient solve_preferred_pitch_ik_client = nodePtr->serviceClient<ik_solver_service::SolvePreferredPitchIK>(
	      "solve_preferred_pitch_ik");

  ik_solver_service::SolvePreferredPitchIK pp_srv;

  pp_srv.request.preferred_pitch = 0; 
  double width = pow((height - 0.03) / (0.44 - 0.03),2) * (0.23 - 0.39) + 0.39;
  pp_srv.request.des_position[0] = 0.0;
  pp_srv.request.des_position[1] = width;
  pp_srv.request.des_position[2] = height;
  pp_srv.request.des_normal[0] = -sin(DEG2RAD(pitch + 90)); 
  pp_srv.request.des_normal[1] = 0.0;
  pp_srv.request.des_normal[2] = cos(DEG2RAD(pitch + 90));

  double cmdPoint[5];

  if (solve_preferred_pitch_ik_client.call(pp_srv))
  {
    for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
	cmdPoint[i] = pp_srv.response.joint_angles[i];
	printf("%f\t", pp_srv.response.joint_angles[i]);
      }
      cmdPoint[0] += yaw;
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", pp_srv.response.feasible,
	      pp_srv.response.arm_to_front, pp_srv.response.arm_bended_up, pp_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_preferred_pitch_ik");
    return;
  }
  
  /*//Create transform from sensor to robot base
  transform.setOrigin( tf::Vector3(sensorOffset.x(), sensorOffset.y(), 0.0) );
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "xtion"));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "baseGoal", "sensorGoal"));
  //-----------------------*/
  
  const clock_t start = std::clock();
  
  brics_actuator::JointPositions command;
  vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(5);
  
  std::stringstream jointName;
  
  for (int i = 0; i < 5; ++i) {
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    
    armJointPositions[i].joint_uri = jointName.str();
    
    armJointPositions[i].value = cmdPoint[i];
    
    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
  
  command.positions = armJointPositions;
  
  arm_pub_pos.publish(command);
  sleep(0.1);
  arm_pub_pos.publish(command);
  
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_hub");
  
  ros::NodeHandle node;

  ros::Rate rate(10.0);
  
  arm_pub_pos = node.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 15);
  ros::Publisher stop_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 15);
  ros::Publisher switch_pub = node.advertise<std_msgs::Bool>("/cloud_throttle", 15);
  
  double height;
  double theta;
  
  std_msgs::Bool cloud_throttle;
  std_srvs::Empty empty_srv;
  
  sleep(2);
  
  //moveArmNavigate();
  
  cout << "Enter Initial Sensor Pose" << endl;
  cin >> camHeight;
  cin >> camPitch;
  
  moveArm(&node, camHeight, camPitch, 0);
  
  sleep(4);
  
  cloud_throttle.data = false;
  switch_pub.publish(cloud_throttle);
  
  MoveBaseClient ac("move_base", true);
  
  while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  tf::TransformListener listener;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::StampedTransform stransform;
  tf::Quaternion q;
  
  tf::Transform modOffset;
  tf::Transform modTransform;
  
  ros::Subscriber sub = node.subscribe("/nbv", 15, nbv_cb);

  ros::Publisher nbv_pub = node.advertise<std_msgs::Int32>("/nbv_iter", 15);
  
  int wait;
  cout << "Ready to begin?" << endl;
  cin >> wait;
  
  std_msgs::Int32 nbv_req;
  
  while (ros::ok())
  {
    nbv_pub.publish<std_msgs::Int32>(nbv_req);
    cout << "Publishhhhh" << endl;
    ros::spinOnce();
    
    ROS_INFO("Publishing NBV Request");
    ROS_INFO("Waiting for a target (to destroy)");
    
    while (!nbvReceived && ros::ok())
    {
      ros::spinOnce();
    }
    
    if (!stationaryNBV)
    {
      ROS_INFO("Positioning Sensor");
    
      cloud_throttle.data = false;
      switch_pub.publish(cloud_throttle);
      ros::service::call<std_srvs::Empty>("/rtabmap/pause", empty_srv);
      
      cout << "Moving to " << camHeight << " : " << camPitch << endl;
      moveArm(&node, 0.4, -30, 0);
      sleep(3);
      
      cloud_throttle.data = true;
      switch_pub.publish(cloud_throttle);
      ros::service::call<std_srvs::Empty>("/rtabmap/resume", empty_srv);
    }
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    nbvReceived = false;
    
    ac.waitForResult();
    
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base moved 1 meter forward");
      nbv_req.data = 1;
    }
    else
    {
      nbv_req.data = 2;
      ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
    
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    
    /*for (int i = 0; i < 50; i++)
    {
      stop_pub.publish(twist);
      rate.sleep();
    }*/
    
    cloud_throttle.data = false;
    switch_pub.publish(cloud_throttle);
    ros::service::call<std_srvs::Empty>("/rtabmap/pause", empty_srv);
    moveArm(&node, camHeight, camPitch, 0);
    
    const clock_t start = clock();
    
    ROS_INFO("Let's Pause for Just a Moment");
    
    /*while (double(clock() - start) / CLOCKS_PER_SEC < 3)
    {
      stop_pub.publish(twist);
      rate.sleep();
    }*/
    
    sleep(3);
    
    ROS_INFO("Resume Mapping");
    cloud_throttle.data = true;
    switch_pub.publish(cloud_throttle);
    ros::service::call<std_srvs::Empty>("/rtabmap/resume", empty_srv);
    
    ROS_INFO("Take 10 s to map");
    sleep(10);
    //cin >> wait;

//       if (nbvReceived && !goalRunning)
//       {
// 	ROS_INFO("Sending goal");
// 	ac.sendGoal(goal);
// 	nbvReceived = false;
// 	goalRunning = true;
//       }
//       if (goalRunning)
//       {
// 	if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
// 	{
// 	  //cout << "Active" << endl;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
// 	{
// 	  //cout << "Pending" << endl;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
// 	{
// 	  cout << "Success" << endl;
// 	  goalRunning = false;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
// 	{
// 	  cout << "Aborted" << endl;
// 	  goalRunning = false;
// 	}
// 	else if (ac.getState() == actionlib::SimpleClientGoalState::LOST)
// 	{
// 	  cout << "Lost" << endl;
// 	  goalRunning = false;
// 	}
//       }
//       ros::spinOnce();
//     }
//     

  }
  
  return 0;
}