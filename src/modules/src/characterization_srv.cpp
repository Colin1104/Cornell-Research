#include <ros/ros.h>

#include <algorithm>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <random>
#include <stdlib.h>
#include <../../opt/ros/indigo/include/ros/subscriber.h>
     
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/math/Pose6D.h>
#include <octomap/math/Quaternion.h>
#include <octomap/math/Vector3.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <octomap_ros/conversions.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

 #include <tf/message_filter.h>
 #include <message_filters/subscriber.h>

#include <modules/character_req.h>
#include <modules/region_req.h>

using namespace std;
using namespace grid_map;

bool debug = false;
int cloud_gate = 0;
octomap::Pointcloud octomap_cloud;
sensor_msgs::PointCloud2 cloud;
tf::StampedTransform sensorToWorldTf;

octomap_msgs::Octomap octomapMsg;
octomap::OcTree* tree;

GridMap* mapPt;
GridMap testMap;

ros::Publisher grid_pub;
ros::Publisher octomap_pub;
ros::Publisher pose_pub;

tf::TransformListener *listenerz;

ros::ServiceClient client;

float robotRad = 0.1;

const vector<float> gaussLedgeHigh = {1, 0.17, 0.01};
const vector<float> gaussLedgeLow = {1, 0.0, 0.01};
const vector<float> logLedgeMid = {2, 0.215, 200};

const vector<vector<float>> gaussLedgeHighRow = {gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh, gaussLedgeHigh};
const vector<vector<float>> gaussLedgeLowRow = {gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow, gaussLedgeLow};
const vector<vector<float>> logLedgeMidRow = {logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid, logLedgeMid};

const vector<vector<vector<float>>> bridge_model = {
  gaussLedgeHighRow,
  gaussLedgeHighRow,
  gaussLedgeHighRow,
  logLedgeMidRow,
  gaussLedgeLowRow,
  gaussLedgeLowRow,
  gaussLedgeLowRow
};

void cloud_cb(sensor_msgs::PointCloud2::ConstPtr msg)
{
  if (cloud_gate > 0)
  {
    ROS_INFO("Cloud received");
    tf::TransformListener tf_listener;
    cloud = *msg;
    
    octomap::pointCloud2ToOctomap(cloud, octomap_cloud);
    
    cloud_gate = 0;
    ROS_INFO("Cloud Processed");
  }
}

bool region_req_cb(modules::region_req::Request &req, modules::region_req::Response &res)
{
  ROS_INFO("Region Request Received");
    
  //GridMap map = testMap;
  
  Position point(req.request.position.x, req.request.position.y);
  if (testMap.isInside(point))
  {
    cout << "Region: " << testMap.atPosition("region", point) << endl;
    res.region.data = int(testMap.atPosition("region", point));
    return true;
  }
  else
  {
    ROS_INFO("Requested point is outside map");
    return false;
  }
}

bool character_req_cb(modules::character_req::Request  &req, modules::character_req::Response &res)
{
  ROS_INFO("Characterization Request Received");
  
  octomap::OcTree* tree;
  
  if (debug)
  {
    tree = new octomap::OcTree("debug_octomap.bt");
  }
  else
  {
    octomap_cloud.clear();
    
    cloud_gate = 1;
    
    while (cloud_gate > 0)
    {
      ros::spinOnce();
    }
    
    ROS_INFO("Populate Octomap");
    
    tree = new octomap::OcTree(0.02);
    
    tree->clear();
    if (cloud.header.frame_id == "map")
    {
      tree->insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), -1, false, true);
    }
    else
    {
      tf::TransformListener tf_listener;
    
      while (ros::ok())
      {
	try {
	  ros::Time gtime = ros::Time::now();
	  listenerz->waitForTransform("map", "camera_rgb_optical_frame", gtime, ros::Duration(2));
	  listenerz->lookupTransform("map", "camera_rgb_optical_frame", gtime, sensorToWorldTf);
	  ROS_INFO("Transform Found");
	  break;
	} catch(tf::TransformException& ex){
	  ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
	}
      }
      
      //octomap::OcTree tree(0.02);
      
      octomap::pose6d sensorPose = octomap::poseTfToOctomap(sensorToWorldTf);
      tree->insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), sensorPose, -1, false, true);
    }
  }
  
  ROS_INFO("Octomap populated");
  
  GridMap map({"elevation", "free", "character", "likelihood", "orientation", "region"});
  map.setFrameId("map");
  
  grid_map::Position3 min_bound;
  grid_map::Position3 max_bound;
  tree->getMetricMin(min_bound(0), min_bound(1), min_bound(2));
  tree->getMetricMax(max_bound(0), max_bound(1), max_bound(2));
  bool check = GridMapOctomapConverter::fromOctomap(*tree, "elevation", map, &min_bound, &max_bound);
  if (!check) {
    ROS_ERROR("Failed to call convert Octomap.");
    return false;
  }
  
  ROS_INFO("Begin Grid Map Processing");
  
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    Position position;   
    map.getPosition(*it, position);
    
    float height = map.at("elevation", *it);
    
    map.at("free", *it) = 1;
    map.at("character", *it) = 1;
    map.at("region", *it) = 0;
    for (grid_map::CircleIterator circleIt(map, position, robotRad); !circleIt.isPastEnd(); ++circleIt)
    {
      float tHeight = map.at("elevation", *circleIt);
      if (fabs(height - tHeight) > 0.04 || isnan(height))
      {
	map.at("free", *it) = 0;
	map.at("character", *it) = 0;
	map.at("region", *it) = -1;
	break;
      }
    }
  }
  
  int region = 1;
  vector<vector<int>> matches;
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {    
    int cellRegion = int(map.at("region", *it));
    
    if (cellRegion < 0) continue;
    
    //cout << "cellRegion: " << cellRegion << endl;
    if (cellRegion == 0)
    {
      cellRegion = region++;
      map.at("region", *it) = cellRegion;
      //cout << "New Region: " << map.at("region", *it) << endl;
    }
    
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
	int nRegion = int(map.at("region", *it + Index(i,j)));
	if (nRegion < 0 || nRegion == cellRegion)
	{
	  continue;
	}
	if (nRegion == 0)
	{
	  map.at("region", Index(i,j) + *it) = cellRegion;
	}
	else
	{
	  vector<int> match = {min(cellRegion, nRegion), max(cellRegion, nRegion)};
	  bool newFlag = true;
	  
	  for (vector<vector<int>>::iterator matchIt = matches.begin(); matchIt != matches.end(); matchIt++)
	  {
	    if ((*matchIt)[0] == match[0] && (*matchIt)[1] == match[1])
	    {
	      newFlag = false;
	      break;
	    }
	  }
	  if (newFlag) matches.push_back(match);
	}
      }
    }
  }
  //cout << matches.size() << endl;
  for (vector<vector<int>>::iterator matchIt = matches.begin(); matchIt != matches.end(); matchIt++)
  {
    /*cout << "Match List:" << endl;
    for (vector<vector<int>>::iterator printIt = matches.begin(); printIt != matches.end(); printIt++)
    {
      cout << (*printIt)[0] << ", " << (*printIt)[1] << endl;
    }*/
    
    for (GridMapIterator iter(map); !iter.isPastEnd(); ++iter)
    {
      vector<int> match = *matchIt;
      if (int(map.at("region", *iter)) == match[1])
      {
	map.at("region", *iter) = match[0];
      }
    }
    
    for (vector<vector<int>>::iterator updateIt = matchIt + 1; updateIt != matches.end(); updateIt++)
    {
      if ((*updateIt)[0] == (*matchIt)[1]) (*updateIt)[0] = (*matchIt)[0];
      if ((*updateIt)[1] == (*matchIt)[1]) (*updateIt)[1] = (*matchIt)[0];
    }
  }
  
  float mapRes = map.getResolution();
  
  ROS_INFO("Free space processed, characterizing");
  int feature_count = 0;
  
  //Iterate over non-free cells to characterize
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    if (map.at("free", *it) == 1)
    {
      continue;
    }
    
    map.at("character", *it) = 0;
    
    Position cellPos;
    map.getPosition(*it, cellPos);
    
    double maxLikelihood = 0;
    double maxThetaD = 0;
    
    //Iterate over orientations to achieve rotational invariance
    for (double thetaD = 0; thetaD < 360; thetaD += 5)
    {
      double theta = DEG2RAD(thetaD);
      //Check for free endpoints
      Position offset(double(6 * mapRes * cos(theta)), double(6 * mapRes * sin(theta)));
      
      if (!map.isInside(cellPos + offset) || !map.isInside(cellPos - offset)
	|| map.atPosition("free", cellPos + offset) != 1 || map.atPosition("free", cellPos - offset) != 1)
      {
	continue;
      }
      
      //ROS_INFO("Found candidate characterization point");
      //map.atPosition("debug", cellPos) = 3;
      
      //Iterate over feature grid
      double likelihood = 1;
      //cout << bridge_model.size() << " < " << bridge_model[0].size() << endl;
      for (int i = 0; i < bridge_model.size(); i++)
      {
	for (int j = 0; j < bridge_model[i].size(); j++)
	{
	  double x = j - double(bridge_model[i].size() - 1) / 2;
	  double y = double(bridge_model.size() - 1) / 2 - i;
	  
	  //cout << j << ", " << i << " : " << x << ", " << y << endl;
	  
	  Position featureOff(double(x * mapRes * sin(theta) + y * mapRes * cos(theta)), double(-x * mapRes * cos(theta) + y * mapRes * sin(theta)));
	  
	  if (!map.isInside(cellPos + featureOff) || isnan(map.atPosition("elevation", cellPos + featureOff)))
	  {
	    likelihood *= 0;
	    break;
	  }
	  
	  float height = map.atPosition("elevation", cellPos + featureOff);
	  
	  switch(int(bridge_model[i][j][0]))
	  {
	    double prob;
	    case 0 : break;
	    case 1 : prob = exp(-pow(bridge_model[i][j][1] - height, 2) / (2 * bridge_model[i][j][2]));
		      likelihood *= prob;
		      //ROS_INFO("Gaussian: ");
		      //cout << "Height, tHeight, Likelihood: " << height << ", " << bridge_model[i][j][1] << ", " << prob << endl;
		      break;
	    case 2 : prob = 1 / (1 + exp(-bridge_model[i][j][2] * (bridge_model[i][j][1] - height)));
		      likelihood *= prob;
		      //ROS_INFO("Logistic: ");
		      //cout << "Height, tHeight, Likelihood: " << height << ", " << bridge_model[i][j][1] << ", " << prob << endl;
		      break;
	  }
	  
	  /*map.atPosition("debug", cellPos - featureOff) = 3;
      
	  grid_map_msgs::GridMap message;
	  grid_map::GridMapRosConverter::toMessage(map, message);
	  grid_pub.publish(message);*/
	  
	  /*if (likelihood < 0.5)
	  {
	    likelihood = 0;
	    break;
	  }*/
	}
	
	/*if (likelihood < 0.5)
	{
	  likelihood = 0;
	  break;
	}*/
      }
      
      //cout << likelihood << endl << endl << endl;
      
      if (likelihood > maxLikelihood)
      {
	maxLikelihood = likelihood;
	maxThetaD = thetaD;
      }
    }
    
    if (maxLikelihood > 0.001)
    {
      cout << "Max Likelihood: " << maxLikelihood << endl;
      cout << "Max Theta: " << maxThetaD << endl;
      
      if (maxLikelihood > 0.01)
      {
	geometry_msgs::PoseArray features;
	geometry_msgs::Pose feature_pose;
	feature_pose.position.x = cellPos.x();
	feature_pose.position.y = cellPos.y();
	feature_pose.position.z = 0.2;
	
	tf::Quaternion q;
	q.setRPY(0, 0, DEG2RAD(maxThetaD));
	geometry_msgs::Quaternion quat;
	tf::quaternionTFToMsg(q, quat);
	feature_pose.orientation = quat;
	
	res.poses.poses.push_back(feature_pose);
	
	Position offset(double(6 * mapRes * cos(DEG2RAD(maxThetaD))), double(6 * mapRes * sin(DEG2RAD(maxThetaD))));
	
	res.region1.data.push_back(int(map.atPosition("region", cellPos - offset)));
	res.region2.data.push_back(int(map.atPosition("region", cellPos + offset)));
	
	//cout << "Regions: " << int(map.atPosition("region", cellPos - offset)) << ", " << int(map.atPosition("region", cellPos + offset)) << endl;
	
	geometry_msgs::PoseStamped feature_stamped;
	feature_stamped.header.frame_id = "map";
	feature_stamped.pose = feature_pose;
	pose_pub.publish(feature_stamped);
	
	map.at("likelihood", *it) = maxLikelihood;
	
	map.atPosition("character", cellPos) = 2;
	map.atPosition("orientation", cellPos) = DEG2RAD(maxThetaD);
	
	map.atPosition("character", cellPos - offset) = 3;
	/*map.atPosition("likelihood", cellPos + offset) = 3;
	
	grid_map_msgs::GridMap message;
	grid_map::GridMapRosConverter::toMessage(map, message);
	grid_pub.publish(message);
	
	//int input;
	//cin >> input;*/
      }
    }
  }
  
  ROS_INFO("Map Processed");
  //mapPt = &map;
  testMap = map;
  
  ROS_INFO("Map Updated");
  
  //tree->writeBinary("ledge_octomap.bt");
  
  /*octomap_msgs::Octomap oct_msg;
  octomap_msgs::fullMapToMsg(*tree, oct_msg);
  oct_msg.header.frame_id = "map";
  octomap_pub.publish(oct_msg);*/
  
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, {"elevation", "likelihood", "character", "orientation", "region"}, message);  
  grid_pub.publish(message);
  
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "character_req");
  
  ros::NodeHandle node;
  
  ros::ServiceServer service = node.advertiseService("characterize", character_req_cb);
  ros::ServiceServer region_service = node.advertiseService("regionize", region_req_cb);
  
  ros::Subscriber cloud_sub = node.subscribe("/minnow/camera/depth_registered/points", 1, cloud_cb);
  
  grid_pub = node.advertise<grid_map_msgs::GridMap>("/the_heights", 1);
  octomap_pub = node.advertise<octomap_msgs::Octomap>("/my_octomap", 1);
  pose_pub = node.advertise<geometry_msgs::PoseStamped>("/feature_pose", 1);
  
  tf::TransformListener tf_listener;
  
  listenerz = &tf_listener;
  
  ros::Rate rate(0.1);
  
  ros::spin();
}

