#include <ros/ros.h>
#include <bits/stdc++.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <env_characterization/classify_map.h>

#include <cmath>
#include <ctime>
#include <chrono>

using namespace grid_map;
using namespace std;

typedef vector<vector<vector<bool>>> gridSubMap;
typedef vector<gridSubMap> thetaSubMap;
typedef vector<thetaSubMap> configMap;

grid_map_msgs::GridMap grid_msg;

octomap::Pointcloud octomap_cloud;
sensor_msgs::PointCloud2 cloud;
tf::StampedTransform sensorToWorldTF;

octomap_msgs::Octomap mapMsg;
bool map_received = false;
bool cloud_received = false;
bool grid_received = false;
bool debug = false;

tf::TransformListener *listener;

class Config
{
public:
  bool omni;		//Is config steerable?
  vector<bool> obs;	//List of features that comprise obstacles
  int w;		//Width of robot footprint
  int h;		//Height of robot footprint
  int r;		//Radius of robot footprint (omni)
  
  Config(bool omni, vector<bool> obs, int h, int w, int r)
  {
    this->omni = omni;
    this->obs = obs;
    this->w = w;
    this->h = h;
    this->r = r;
  }
};

void map_cb(const octomap_msgs::OctomapConstPtr& map)
{
  cout << ".";
  mapMsg = *map;
  map_received = true;
}

void grid_cb(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_msg = *msg;
  grid_received = true;
}

void cloud_cb(sensor_msgs::PointCloud2::ConstPtr msg)
{
  try
  {
    listener->lookupTransform("map", "camera_rgb_optical_frame", ros::Time(0), sensorToWorldTF);
  }
  catch(tf::TransformException& ex)
  {
    return;
  }
  
  cloud = *msg;
  octomap::pointCloud2ToOctomap(cloud, octomap_cloud);
  cloud_received = true;
}

vector<int> ClassifyFlat(const nav_msgs::OccupancyGrid &occ_grid, int padding, int filterSize, float varThresh)
{
  //0 - obs, 1 - flat
  vector<int> features(occ_grid.data.size(), 0);
  
  for (int i = 0; i < occ_grid.info.height - filterSize; i++)
  {
    for (int j = 0; j < occ_grid.info.width - filterSize; j++)
    {
      float xSqSum = 0;
      float xSum = 0;
      for (int offY = 0; offY < filterSize; offY++)
      {
	for (int offX = 0; offX < filterSize; offX++)
	{
	  int idx = (i + offY) * occ_grid.info.width + j + offX;
	  
	  xSqSum += pow(occ_grid.data[idx], 2);
	  xSum += occ_grid.data[idx];
	}
      }
      
      float var = xSqSum / pow(filterSize, 2) - pow(xSum / pow(filterSize, 2), 2);
      
      if (var < varThresh)
      {
	for (int offY = 0; offY < filterSize; offY++)
	{
	  for (int offX = 0; offX < filterSize; offX++)
	  {
	    int idx = (i + offY) * occ_grid.info.width + j + offX;
	    
	    features[idx] = 1;
	  }
	}
      }
    }
  }
  
  return features;
}

vector<nav_msgs::OccupancyGrid> PartitionMap(const nav_msgs::OccupancyGrid &occ_grid, int size)
{  
  vector<nav_msgs::OccupancyGrid> snippets;
  
  cout << "Input Map Dimensions: " << occ_grid.info.height << " x " << occ_grid.info.width << " : " << occ_grid.data.size() << endl;
  
  for (int i = 0; i < occ_grid.info.height - size; i++)
  {
    for (int j = 0; j < occ_grid.info.width - size; j++)
    {
      int idx = i * occ_grid.info.width + j;
      
      nav_msgs::OccupancyGrid snippet = occ_grid;
      snippet.info.height = size;
      snippet.info.width = size;
      
      geometry_msgs::Pose pos;
      pos.position.x = occ_grid.info.origin.position.x + j * occ_grid.info.resolution;
      pos.position.y = occ_grid.info.origin.position.y + i * occ_grid.info.resolution;
      snippet.info.origin.position = pos.position;
      vector<int8_t> vec;
      for (int k = 0; k < size; k++)
      {
	int basedx = idx + k * occ_grid.info.width;
	vec.insert(vec.end(), &(occ_grid.data[basedx]), &(occ_grid.data[basedx + size]));
      }
      snippet.data = vec;
      
      snippets.push_back(snippet);
    }
  }
  
  return snippets;
}

void bloatRadius(gridSubMap &grid, int cogY, int cogX, int r, int bloatFeat)
{  
  for (int i = -r; i < r; i++)
  {
    //iterate over a circle with radius r
    int chord = sqrt(pow(r, 2) - pow(i, 2));
    for (int j = -chord; j < chord; j++)
    {
      int y = cogY + i;
      int x = cogX + j;
      
      if (y >= 0 && y < grid.size() && x >= 0 && x < grid[y].size())
      {
	grid[y][x][bloatFeat] = 1;
      }
    }
  }
}

void bloatFoot(gridSubMap &grid, int cogY, int cogX, int w, int h, float theta, int bloatFeat)
{
  for (int i = -h / 2; i <= h / 2; i++)
  {
    for (int j = -w / 2; j <= w / 2; j++)
    {
      int x = i * cos(theta) - j * sin(theta) + cogX;
      int y = i * sin(theta) + j * cos(theta) + cogY;
      
      if (y >= 0 && y < grid.size() && x >= 0 && x < grid[y].size())
      {
	grid[y][x][bloatFeat] = 1;
      }
    }
  }
}

configMap bloatMap(const vector<int> &data, int height, int width, const vector<Config> &configs, int nAngles, int nFeats)
{
  //Init config-space map
  configMap planGraph;
  
  for (int c = 0; c < configs.size(); c++)
  {
    int angleSize = configs[c].omni + nAngles * !configs[c].omni;
    planGraph.push_back(
      thetaSubMap(angleSize, gridSubMap(height, vector<vector<bool>>(width, vector<bool>(nFeats, 0)))));
  }
  
  //Iterate over cells
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      int feature = data[i * width + j];
      if (feature <= 0) continue;
      //Iterate over configs
      for (int c = 0; c < configs.size(); c++)
      {
	Config conf = configs[c];
	if (conf.omni)
	{
	  bloatRadius(planGraph[c][0], i, j, conf.r, feature * !conf.obs[feature]);
	}
	else
	{
	  for (int a = 0; a < nAngles; a++)
	  {
	    float theta = float(a) * 2 * 3.141592 / float(nAngles);
	    bloatFoot(planGraph[c][a], i, j, conf.w, conf.h, theta, feature * !conf.obs[feature]);
	  }
	}
      }
    }
  }
  
  return planGraph;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "env_characterization");
  ros::NodeHandle node;
  
  ros::Rate rate(20.0);
  
  ros::Subscriber grid_sub = node.subscribe("/debug_grid", 5, grid_cb);
  ros::Subscriber cloud_sub = node.subscribe("/camera/depth_registered/points", 5, cloud_cb);
  ros::Publisher grid_pub = node.advertise<grid_map_msgs::GridMap>("/elevation", 1);
  ros::Publisher sub_pub = node.advertise<grid_map_msgs::GridMap>("/elevation_sub", 1);
  ros::Publisher occ_pub = node.advertise<nav_msgs::OccupancyGrid>("/input_occ_map", 5);
  ros::Publisher class_pub = node.advertise<nav_msgs::OccupancyGrid>("/classes", 5);
  ros::Publisher bloat_pub = node.advertise<nav_msgs::OccupancyGrid>("/bloated_map", 5);
  ros::Publisher flat_pub = node.advertise<nav_msgs::OccupancyGrid>("/flat_map", 5);
  
  ros::ServiceClient client = node.serviceClient<env_characterization::classify_map>("/classify_map");
  
  listener = new tf::TransformListener;
  
  GridMap grid({"elevation"});
  grid.setFrameId("map");
  
  if (debug)
  {
    cout << "Debug Mode" << endl;
    
    while (!grid_received && ros::ok())
    {
      ros::spinOnce();
    }
    
    cout << "Got Grid" << endl;
    
    GridMapRosConverter::fromMessage(grid_msg, grid);
  }
  else
  {
    while (!cloud_received && ros::ok())
    {
      ros::spinOnce();
    }
    
    cout << "Got cloud" << endl;
  }
  
  octomap::OcTree tree(0.01);
  
  octomap::pose6d sensorPose = octomap::poseTfToOctomap(sensorToWorldTF);
  tree.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), sensorPose, -1, false, true);
  
  GridMapOctomapConverter::fromOctomap(tree, "elevation", grid);
  
  nav_msgs::OccupancyGrid occ_grid;
  GridMapRosConverter::toOccupancyGrid(grid, "elevation", -0.5, 0.5, occ_grid);
  
  int size = 12;
  
  vector<nav_msgs::OccupancyGrid> partitions = PartitionMap(occ_grid, size);
  
  cout << "Partitions: " << partitions.size() << endl;
  
  env_characterization::classify_map srv;
  srv.request.partitions = partitions;
  
  nav_msgs::OccupancyGrid classes = occ_grid;
  
  //int selection = 0;
  //cin >> selection;
  
  vector<int> featureMap;
  
  chrono::steady_clock::time_point start = chrono::steady_clock::now();
  for (int i = 0; i < 1; i++)
  {
    if (client.call(srv))
    {
      cout << "Service Responded" << endl;
      classes.data = srv.response.characters.data;
      for (int i = 0; i < classes.data.size(); i++)
      {
	featureMap.push_back(classes.data[i] > 0);
	//if (classes.data[i] != selection) classes.data[i] = 0;
	classes.data[i] *= 10;
      }
      
      classes.info.height -= size;
      classes.info.width -= size;
      classes.info.origin.position.x += size / 2 * classes.info.resolution;
      classes.info.origin.position.y += size / 2 * classes.info.resolution;
    }
    else
    {
      cout << "Service Failed" << endl;
    }
  }
  chrono::steady_clock::time_point stop = chrono::steady_clock::now();
  
  cout << chrono::duration_cast<chrono::microseconds>(stop - start).count() / 1000000.0 << endl;
  
  //Planning Graph Construction
  //Obstacle candidate list: [flat, ledge]
  Config carCon(true, {0, 1}, 11, 5, 5);
  Config snakeCon(false, {0, 0}, 11, 5, 7);
  vector<Config> configList = {carCon, snakeCon};
  
  configMap planGraph = bloatMap(featureMap, classes.info.height, classes.info.width, configList, 8, 2);
  
  gridSubMap vizGraph = planGraph[1][0];
  
  cout << "Graph Sizes: car angles " << planGraph[0].size() << "; snake angles " << planGraph[1].size() << endl;
  
  vector<int> vizBloatData;
  
  for (int i = 0; i < classes.info.height; i++)
  {
    for (int j = 0; j < classes.info.width; j++)
    {
      vizBloatData.push_back(vizGraph[i][j][1]);
    }
  }
  
  nav_msgs::OccupancyGrid bloat_msg = classes;
  bloat_msg.data = {};
  
  for (int i = 0; i < vizBloatData.size(); i++)
  {
    bloat_msg.data.push_back(vizBloatData[i] * 10);
  }
  
  grid_map_msgs::GridMap grid_msg;
  GridMapRosConverter::toMessage(grid, grid_msg);
  
  while (ros::ok())
  {
    occ_pub.publish(occ_grid);
    class_pub.publish(classes);
    grid_pub.publish(grid_msg);
    bloat_pub.publish(bloat_msg);
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}