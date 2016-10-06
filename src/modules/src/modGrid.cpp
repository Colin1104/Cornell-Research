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

#include <modules/sensorCell.h>
#include <modules/MinHeap.h>
#include <modules/InfoNode.h>

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

bool mapReceived;
Octomap mapMsg;
double res = 0.01;
double rFactor = 1 / res;
ros::Publisher path_pub;
ros::Publisher grid_pub;

int generatePath(string egoTagString);

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

void map_cb(const octomap_msgs::OctomapConstPtr& map)
// Callback for octopap topic
{
  cout << ".";
  mapMsg = *map;
  mapReceived = true;
}

void tag_info_cb(std_msgs::String dataString)
// Callback for tag information topic
{
  string egoTagString = parse(dataString.data, 0); // TODO write me
  generatePath(egoTagString);
}

int main(int argc, char** argv)
{
    cout<<"Starting modGrid"<<endl;
  // Main creates a node that will generate and publish a path when new tag names are received.
  ros::init(argc, argv, "modGrid");
  ros::NodeHandle node;
  ros::Rate rate(10.0);
  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  tf::Transform modOffset;
  tf::Transform modTransform;
  
  ros::Subscriber sub = node.subscribe("/octomap_binary", 15, map_cb);
  path_pub = node.advertise<nav_msgs::Path>("/smorePath", 15);
  grid_pub = node.advertise<nav_msgs::OccupancyGrid>("/nav_map", 15);
  // Subscribe to tag info topic:
  ros::Subscriber tagInfoSub = node.subscribe("/reconf_request", 10, tag_info_cb); 

  while(ros::ok())
  {
      ros::spinOnce();
  }

}

int generatePath(string egoTagString)
{
  tf::TransformListener listener;
  tf::StampedTransform stransform;

  cout << "Waiting for map" << endl;
  
  while (!mapReceived && ros::ok())
  {
    ros::spinOnce();
  }
  
  cout << "Got map" << endl;

  AbstractOcTree* atree = octomap_msgs::msgToMap(mapMsg);
  OcTree* tree = NULL;
  if (atree){
    tree = dynamic_cast<OcTree*>(atree);
  } else {
    ROS_ERROR("Error creating octree from received message");
    if (mapMsg.id == "ColorOcTree")
      ROS_WARN("You requested a binary map for a ColorOcTree - this is currently not supported. Please add -f to request a full map");
  }
  
  sleep(5);
  
  try{
    ros::Time gTime = ros::Time::now();
    /*
    listener.waitForTransform("map", "tag_0", gTime, ros::Duration(15.0));
    listener.lookupTransform("map", "tag_0", gTime, stransform);
    */
    listener.waitForTransform("map", egoTagString, gTime, ros::Duration(15.0));
    listener.lookupTransform("map", egoTagString, gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d origin(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  cout << origin.x() << " : " << origin.y() << " : " << origin.z() << endl;
  
  try{
    ros::Time gTime = ros::Time::now();
    // Goal transform is published by modMap.cpp
    listener.waitForTransform("map", "goal", gTime, ros::Duration(15.0));
    listener.lookupTransform("map", "goal", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d goal(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  cout << "Reach Point: " << goal.x() << " : " << goal.y() << " : " << goal.z() << endl;

  try{
    ros::Time gTime = ros::Time::now();
    // Goal transform is published by modMap.cpp
    listener.waitForTransform("map", "goal1", gTime, ros::Duration(15.0));
    listener.lookupTransform("map", "goal1", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d goal1(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  cout << "Reach Point: " << goal1.x() << " : " << goal1.y() << " : " << goal1.z() << endl;

  try{
    ros::Time gTime = ros::Time::now();
    // Goal transform is published by modMap.cpp
    listener.waitForTransform("map", "goal2", gTime, ros::Duration(15.0));
    listener.lookupTransform("map", "goal2", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d goal2(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  cout << "Reach Point: " << goal2.x() << " : " << goal2.y() << " : " << goal2.z() << endl;
  
  try{
    ros::Time gTime = ros::Time::now();
    // Dock transform is published by modMap.cpp
    listener.waitForTransform("map", "dock", gTime, ros::Duration(15.0));
    listener.lookupTransform("map", "dock", gTime, stransform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  point3d dock(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());
  cout << "Docking Point: " << dock.x() << " : " << dock.y() << " : " << dock.z() << endl;
  
  double robotRad = 0.07;
  
  double minX, minY, minZ, maxX, maxY, maxZ;
  tree->getMetricMin(minX, minY, minZ);
  tree->getMetricMax(maxX, maxY, maxZ);
  point3d minPoint(minX, minY, minZ);
  point3d maxPoint(maxX, maxY, maxZ);
  
  cout << minX << ", " << maxX << ", " << minY << ", " << maxY << endl;
  minPoint = tree->keyToCoord(tree->coordToKey(minPoint));
  minX = minPoint.x();
  minY = minPoint.y();
  minZ = minPoint.z();
  maxPoint = tree->keyToCoord(tree->coordToKey(maxPoint));
  maxX = maxPoint.x();
  maxY = maxPoint.y();
  maxZ = maxPoint.z();
  
  cout << minX << ", " << maxX << ", " << minY << ", " << maxY << endl;
  
  tree->expand();
  
  int rangeX = round((maxX - minX) * rFactor);
  int rangeY = round((maxY - minY) * rFactor);
  
  vector<GridNode> gridMap(rangeX * rangeY, GridNode());
  vector<int> freeMap(rangeX * rangeY, 0);
  
  for (int x = 0; x < rangeX; x++)
  {
    for (int y = 0; y < rangeY; y++)
    {
      //cout << "XY: " << x << ", " << y << endl;
      gridMap[rangeX * y + x].coords = point3d(double(x) * res + minX, double(y) * res + minY, 0);
      
      for (int i = -1; i <= 1; i += 1)
      {
    for (int j = -1; j <= 1; j += 1)
    {
      if ((i != 0 || j != 0) && x + i >= 0 && x + i < rangeX && y + j >= 0 && y + j < rangeY)
      {
        gridMap[rangeX * y + x].neighbors.push_back(&gridMap[rangeX * (y + j) + x + i]);
        gridMap[rangeX * y + x].edges.push_back(sqrt(pow(i, 2) + pow(j, 2)) * res);
        
        //cout << gridMap[rangeX*y + x].neighbors.size() << ", " << gridMap[rangeX*y + x].edges.size() << endl;
      }
    }
      }
    }
  }
  
  cout << "First" << endl;
  
  tree->expand();
  for (OcTree::leaf_iterator iter = tree->begin_leafs(); iter != tree->end_leafs(); iter++)
  {
    if (tree->isNodeOccupied(*iter) && iter.getZ() > 0.05 && iter.getCoordinate().distanceXY(origin) > 0.08)
    {
      int index = int((iter.getCoordinate().y() - minY) / res) * rangeX + (iter.getCoordinate().x() - minX) / res;
      
      gridMap[index].object = true;
      gridMap[index].occupied = true;
      
      for (double offX = -robotRad; offX <= robotRad; offX += res)
      {
    for (double offY = -robotRad; offY <= robotRad; offY += res)
    {
      if (sqrt(pow(offX, 2) + pow(offY, 2)) <= robotRad)
      {
        int xInd = (gridMap[index].coords.x() + offX - minX) * rFactor;
        int yInd = (gridMap[index].coords.y() + offY - minY) * rFactor;
        
        if (xInd >= 0 && xInd < rangeX && yInd >= 0 && yInd < rangeY)
        {
          int offIdx = yInd * rangeX + xInd;
          gridMap[offIdx].occupied = true;
        }
      }
    }
      }
    }
  }  
  cout << "Second" << endl;
  
  cout << rangeX << ", " << rangeY << " : " << int((origin.x() - minX) * rFactor) << ", " << int((origin.y() - minY) * rFactor) << endl;
  
  int origIdx = rangeX * int((origin.y() - minY) / res) + int((origin.x() - minX) / res);
  
  cout << origIdx << "     " << gridMap.size() << endl;
  
  GridNode* originNode = &gridMap[origIdx];
  originNode->cost = 0;
  
  cout << "Orig Node: " << originNode->coords.x() << ", " << originNode->coords.y() << endl;
  
  cout << "Got Here" << endl;
  
  MinHeap heapy;
  
  heapy.Push(originNode);
  
  cout << "And Here" << endl;
  
  while (heapy.GetLength() > 0)
  {
    GridNode* minNode = heapy.Pop();
    
    vector<float>::iterator edgeIt = minNode->edges.begin();
    for (vector<GridNode*>::iterator iter = minNode->neighbors.begin(); iter != minNode->neighbors.end(); iter++, edgeIt++)
    {
      //GridNode* nodePt = *iter; // unused?
      if ((*iter)->occupied == 0 && (*iter)->cost > (minNode->cost + *edgeIt))
      {
    (*iter)->parent = minNode;
    
    if ((*iter)->state == 0)
    {
      (*iter)->cost = minNode->cost + *edgeIt;
      heapy.Push((*iter));
      (*iter)->state = 1;
    }
    else if ((*iter)->state == 1)
    {
      heapy.Update((*iter)->heapIdx, minNode->cost + *edgeIt);
    }
      }
    }
  }
  
  cout << "Dijkstra Path Complete" << endl;
  
  nav_msgs::OccupancyGrid grid;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = "map";
  
  nav_msgs::MapMetaData metas;
  metas.resolution = res;
  metas.origin.position.x = minX;
  metas.origin.position.y = minY;
  metas.origin.position.z = 0.01;
  metas.height = rangeY;
  metas.width = rangeX;
  
  grid.info = metas;
  
  cout << gridMap.size() << endl;
  
  for (vector<GridNode>::iterator iter = gridMap.begin(); iter != gridMap.end(); iter++)
  {    
    if (iter->occupied)
    {
      grid.data.push_back(100);
    }
    else
    {
      grid.data.push_back(0);
    }
  }
  
  grid_pub.publish(grid);
  
  cout << "Map Published" << endl;
  
  GridNode* endNode = &gridMap[rangeX * int((goal.y() - minY) / res) + int((goal.x() - minX) / res)];
  
  cout << "End Node: " << endNode->coords.x() << ", " << endNode->coords.y() << endl;
    
  vector<point3d> waypoints;
  
  while (endNode != NULL)
  {
    //cout << endNode->coords.x() << ", " << endNode->coords.y() << endl;
    //cout << "Cost: " << endNode->cost << endl;
    endNode->cost = 0;
    waypoints.push_back(endNode->coords);
    //cout << endNode->coords.x() << ", " << endNode->coords.y() << endl;
    endNode = endNode->parent;
  }
  
  reverse(waypoints.begin(), waypoints.end());
  
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  
  int state = 0;
  int oldState = -1;
  for (vector<point3d>::iterator iter = waypoints.begin(); iter != waypoints.end(); iter++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = path.header.stamp;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.position.x = iter->x();
    pose.pose.position.y = iter->y();
    pose.pose.position.z = 0.08;
    
    if (path.poses.size() > 0)
    {
      if (iter->x() == path.poses.back().pose.position.x)
      {
    state = 0;
      }
      else if (iter->y() == path.poses.back().pose.position.y)
      {
    state = 1;
      }
      else
      {
    state = 2;
      }
    }
    
    cout << "State, Old State, Size: " << state << " : " << oldState << " : " << path.poses.size() << endl;
    
    if (path.poses.size() > 1 && state == oldState)
    {
      path.poses.pop_back();
      cout << pose.pose.position.x << ", " << pose.pose.position.y << endl;
    }
    
    path.poses.push_back(pose);
    
    oldState = state;
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = path.header.stamp;
  pose.header.frame_id = path.header.frame_id;
  pose.pose.position.x = goal1.x();
  pose.pose.position.y = goal1.y();
  pose.pose.position.z = 0.08;
  path.poses.push_back(pose);

  pose.header.stamp = path.header.stamp;
  pose.header.frame_id = path.header.frame_id;
  pose.pose.position.x = goal2.x();
  pose.pose.position.y = goal2.y();
  pose.pose.position.z = 0.08;
  path.poses.push_back(pose);
  
  pose.header.stamp = path.header.stamp;
  pose.header.frame_id = path.header.frame_id;
  pose.pose.position.x = dock.x();
  pose.pose.position.y = dock.y();
  pose.pose.position.z = 0.08;
  //path.poses.push_back(pose);
  
  cout << "Path Message Generated" << endl;
  
  path_pub.publish<nav_msgs::Path>(path);
  ros::spinOnce();
  
  cout << "Path Message Published" << endl;
  
  return 0;
}


