#include <ros/ros.h>
#include <bits/stdc++.h>
#include <queue>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_core/SubmapGeometry.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32MultiArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
//#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeIterator.hxx>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <env_characterization/classify_map.h>
#include <env_characterization/PathNode.h>
#include <env_characterization/PathNodeArray.h>
#include <env_characterization/Feature.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cmath>
#include <ctime>
#include <chrono>

#include <iomanip>

using namespace grid_map;
using namespace std;

typedef vector<vector<vector<bool>>> gridSubMap;
typedef vector<gridSubMap> thetaSubMap;
typedef vector<thetaSubMap> configMap;

grid_map_msgs::GridMap grid_msg;
grid_map_msgs::GridMap snip_msg;
GridMap* gridPtr;
ros::ServiceClient* clientPtr;

octomap::Pointcloud octomap_cloud;
sensor_msgs::PointCloud2 cloud;
tf::StampedTransform sensorToWorldTF;

octomap_msgs::Octomap mapMsg;
bool map_received = false;
bool cloud_received = false;
bool grid_received = false;
bool debug = false;

ros::Publisher snip_pub;

tf::TransformListener *listener;

struct NodeSpec
{
  int c;
  int th;
  int i;
  int j;
  
  NodeSpec()
  {
    
  }
  
  NodeSpec(int c_, int th_, int i_, int j_)
  {
    c = c_;
    th = th_;
    i = i_;
    j = j_;
  }
};

struct Link
{
  pair<NodeSpec, NodeSpec> vertices;
  int action;
  float cost;
  
  Link(pair<NodeSpec, NodeSpec> v, int a, float c)
  {
    vertices = v;
    action = a;
    cost = c;
  }
};

class Pose
{
public:
  float x, y, theta;
  
  Pose(int x, int y, int theta)
  {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }
  
  Pose()
  {
  }
  
  Pose operator+(const Pose &p)
  {
    return Pose(this->x + p.x, this->y + p.y, this->theta + p.theta);
  }
};

class Config
{
public:
  bool omni;			//Is config steerable?
  vector<bool> obs;		//Obstacle status for each feature, starting with flat
  vector<vector<float>> costs;	//List of costs for each [action, feature]
  int w;			//Width of robot footprint
  int h;			//Height of robot footprint
  int r;			//Radius of robot footprint (omni)
  
  Config(bool omni, vector<bool> obs, vector<vector<float>> costs, int h, int w, int r)
  {
    this->omni = omni;
    this->obs = obs;
    this->costs = costs;
    this->w = w;
    this->h = h;
    this->r = r;
  }
  
  Config()
  {
    
  }
  
  vector<Pose> getActions(int theta)
  {
    vector<Pose> acts;
    if (this->omni)
    {
      //Eight-Connected Neighbors
      acts = {Pose(-1, -1, 0), Pose(-1, 0, 0), Pose(-1, 1, 0), Pose(0, -1, 0), Pose(0, 1, 0),
	Pose(1, -1, 0), Pose(1, 0, 0), Pose(1, 1, 0)};
    }
    else
    {
      //Forward-Backward Neighbors
      switch (theta)
      {
	case 0:
	  acts = {Pose(-1, 0, 0), Pose(1, 0, 0)};//, Pose(-5, 0, 0), Pose(5, 0, 0)};
	  break;
	case 1:
	  acts = {Pose(-2, -1, 1), Pose(2, 1, 1)};//, Pose(-4, -2, 1), Pose(4, 2, 1)};
	  break;
	case 2:
	  acts = {Pose(-1, -1, 2), Pose(1, 1, 2)};//, Pose(-4, -4, 2), Pose(4, 4, 2)};
	  break;
	case 3:
	  acts = {Pose(-1, -2, 3), Pose(1, 2, 3)};//, Pose(-2, -4, 3), Pose(2, 4, 3)};
	  break;
	case 4:
	  acts = {Pose(0, -1, 4), Pose(0, 1, 4)};//, Pose(0, -5, 4), Pose(0, 5, 4)};
	  break;
	case 5:
	  acts = {Pose(1, -2, 5), Pose(-1, 2, 5)};//, Pose(2, -4, 5), Pose(-2, 4, 5)};
	  break;
	case 6:
	  acts = {Pose(1, -1, 6), Pose(-1, 1, 6)};//, Pose(4, -4, 6), Pose(-4, 4, 6)};
	  break;
	case 7:
	  acts = {Pose(2, -1, 7), Pose(-2, 1, 7)};//, Pose(4, -2, 7), Pose(-4, 2, 7)};
	  break;
	default:
	  acts = {};
	  break;
      }
    }
    
    return acts;
  }
};

class Node
{
public:
  Pose pose;
  int config;
  Node* parent;
  int action;
  float cost;
  vector<int> neighbs;
  vector<float> edgeCosts;
  vector<int> actions;
  
  Node(Pose pose, int config)
  {
    this->pose = pose;
    this->config = config;
    cost = numeric_limits<float>::infinity();
    parent = NULL;
    action = 0;
  }
  
  Node(Pose pose, int config, vector<int> neighbs, vector<float> edgeCosts, vector<int> actions)
  {
    this->pose = pose;
    this->config = config;
    this->neighbs = neighbs;
    this->edgeCosts = edgeCosts;
    this->actions = actions;
    cost = numeric_limits<float>::infinity();
    parent = NULL;
    action = 0;
  }
};

class pathNode
{
public:
  int id;
  float cost;
  int pID;
  
  pathNode()
  {
    id = -1;
    cost = numeric_limits<float>::infinity();
    pID = -1;
  }
  
  pathNode(int id)
  {
    this->id = id;
    cost = numeric_limits<float>::infinity();
    pID = -1;
  }
  
  pathNode(int id, float cost)
  {
    this->id = id;
    this->cost = cost;
    this->pID = -1;
  }
  
  pathNode(int id, int pID, float cost)
  {
    this->id = id;
    this->cost = cost;
    this->pID = pID;
  }
};

typedef vector<vector<int>> gridNodeMap;
typedef vector<gridNodeMap> thetaNodeMap;
typedef vector<thetaNodeMap> configNodeMap;

void click_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  cout << "Entered click_cb" << endl;
  geometry_msgs::Pose pose = msg->pose.pose;
  double yaw = tf::getYaw(pose.orientation);
  
  cout << "Height at (" << pose.position.x << ", " << pose.position.y << "): " << endl;
  Position pos(pose.position.x, pose.position.y);
  if (gridPtr->isInside(pos))
  {
    cout << "Wellllllllllllp" << endl;
    cout << gridPtr->atPosition("elevation", pos) << endl;
    
    std::cout << std::setprecision(2) << std::fixed;
    
    bool success;
    GridMap snip = gridPtr->getSubmap(pos, Length(17 * gridPtr->getResolution(), 17 * gridPtr->getResolution()), success);
    
    GridMap snip_rot = snip;
    
    Eigen::Rotation2Dd t(0); //90 * M_PI / 180 + yaw);
    int rowCt = 0;
    grid_map::Matrix& data = snip_rot["elevation"];
    for (GridMapIterator it(snip_rot); !it.isPastEnd(); ++it)
    {
      int i = it.getLinearIndex();
      
      Position position;
      snip_rot.getPosition(*it, position);
      
      position -= snip_rot.getPosition();
      
      /*Eigen::Vector2d vec = position - snip_rot.getPosition();
      Eigen::Vector2d pos_rot = t * vec;
      
      position = snip_rot.getPosition() + pos_rot;
      
      if (gridPtr->isInside(position))
      {
	snip_rot.at("elevation", *it) = gridPtr->atPosition("elevation", position);
      }*/
      
      cout << data(i) << ", ";
      //cout << "(" << position[0] << ", " << position[1] << "), ";
      if (++rowCt % 18 == 0) cout << endl;
    }
    
    cv::Mat originalImage;
    GridMapCvConverter::toImage<uint8_t, 1>(snip_rot, "elevation", CV_8UC1, 0.0, 1.0, originalImage);
    string layer = "image";
    GridMapCvConverter::addLayerFromImage<uint8_t, 1>(originalImage, layer, snip_rot);
    cv::imwrite("/home/jonathan/catkin_ws/snippet_img.png", originalImage);
    
    GridMapRosConverter::toMessage(snip_rot, snip_msg);
    
    bool isSuccess;
    SubmapGeometry geom(*gridPtr, pos, Length(17 * gridPtr->getResolution(), 17 * gridPtr->getResolution()), isSuccess);
    
    if (isSuccess)
    {
      std_msgs::Float32MultiArray snip;
      
      int rowCt = 0;
      for (SubmapIterator iter(geom); !iter.isPastEnd(); ++iter)
      {
	snip.data.push_back(gridPtr->at("elevation", *iter));
	
	cout << gridPtr->at("elevation", *iter) << ", ";
	if (++rowCt % 18 == 0) cout << endl;
	
	/*gridPtr->at("elevation", *iter) = 0.0;
	GridMapRosConverter::toMessage(*gridPtr, snip_msg);
	ros::Rate(10).sleep();
	snip_pub.publish(snip_msg);*/
      }
    
      env_characterization::classify_map srv;
      srv.request.partitions.push_back(snip);
      
      if (clientPtr->call(srv))
      {
	env_characterization::Feature feat = srv.response.characters[0];
	cout << "Feature: " << feat.feature << endl;
	cout << "Param: " << feat.param << endl;
      }
      else
      {
	cout << "Service failed" << endl;
      }
    }
    else
    {
      cout << "Invalid submap" << endl;
    }
  }
  else
  {
    cout << "isn't valid" << endl;
  }
}

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

void Flattenize(GridMap &grid, vector<env_characterization::Feature> *features, double padding, int rFilter, float varThresh)
{  
  bool isSuccess;
  Length len = grid.getLength() - Length(padding, padding);
  cout << len[0] << "..." << len[1] << endl;
  SubmapGeometry geom(grid, grid.getPosition(), grid.getLength() - Length(padding, padding), isSuccess);
  
  if (isSuccess)
  {
    int idx = 0;
    for (SubmapIterator iter(grid, Index(10, 10), grid.getSize() - Size(20, 20)); !iter.isPastEnd(); ++iter)
    {
      Position cellPos;
      grid.getPosition(*iter, cellPos);
      
      float xSqSum = 0;
      float xSum = 0;
      int sampleCt = 0;
      
      double radius = rFilter * grid.getResolution();
      
      //cout << "Position: " << cellPos[0] << ", " << cellPos[1] << endl;

      for (grid_map::CircleIterator iterator(grid, cellPos, radius); !iterator.isPastEnd(); ++iterator)
      {
	float val = grid.at("elevation", *iterator);
	
	xSqSum += pow(val, 2);
	xSum += val;
	sampleCt++;
      }
      
      float var = xSqSum / sampleCt - pow(xSum / sampleCt, 2);
      
      if (var < varThresh || sampleCt == 0)
      {
	(*features)[idx].feature = 0;
	grid.at("feature", *iter) = 0;
      }
      
      idx++;
    }
  }
  else
  {
    cout << "Invalid submap" << endl;
  }
}

vector<int> ClassifyFlat(const nav_msgs::OccupancyGrid &occ_grid, int padding, int rFilter, float varThresh)
{
  //0 - obs, 1 - flat
  vector<int> features(occ_grid.data.size(), 0);
  
  for (int i = 0; i < occ_grid.info.height; i++)
  {
    for (int j = 0; j < occ_grid.info.width; j++)
    {
      float xSqSum = 0;
      float xSum = 0;
      int sampleCt = 0;
      for (int offY = -rFilter; offY < rFilter; offY++)
      {
	int chord = sqrt(pow(rFilter, 2) - pow(offY, 2));
	for (int offX = -chord; offX < chord; offX++)
	{
	  int y = i + offY;
	  int x = j + offX;
	  
	  if (y < 0 || y >= occ_grid.info.height || x < 0 || x >= occ_grid.info.width) continue;
	  
	  int idx = (i + offY) * occ_grid.info.width + j + offX;
	  
	  float val = float(occ_grid.data[idx]) / 100.0 - 0.5;
	  if (occ_grid.data[idx] == -1) val = 0;
	  
	  xSqSum += pow(val, 2);
	  xSum += val;
	  sampleCt++;
	}
      }
      
      float var = xSqSum / sampleCt - pow(xSum / sampleCt, 2);
      
      if (var < varThresh || sampleCt == 0)
      {
	features[i * occ_grid.info.width + j] = 0;
	
	/*for (int offY = 0; offY < filterSize; offY++)
	{
	  for (int offX = 0; offX < filterSize; offX++)
	  {
	    int idx = (i + offY) * occ_grid.info.width + j + offX;
	    
	    features[idx] = 0;
	  }
	}*/
      }
    }
  }
  
  return features;
}

vector<std_msgs::Float32MultiArray> Snippetize(GridMap &grid, double padding)
{
  vector<std_msgs::Float32MultiArray> snippets;
  
  int snipCt = 0;
  for (SubmapIterator iter(grid, Index(10, 10), grid.getSize() - Size(20, 20)); !iter.isPastEnd(); ++iter)
  {
    Position cellPos;
    grid.getPosition(*iter, cellPos);
    
    if (int(grid.at("feature", *iter)) == -1)
    {      
      std_msgs::Float32MultiArray snip;
      for (SubmapIterator snipIt(grid, *iter - Index(9, 9), Size(18, 18)); !snipIt.isPastEnd(); ++snipIt)
      {
	snip.data.push_back(grid.at("elevation", *snipIt));
      }
      
      snippets.push_back(snip);
      snipCt++;
    }
  }
  
  cout << "snipCt: " << snipCt << endl;
  
  return snippets;
}

vector<nav_msgs::OccupancyGrid> PartitionMap(const nav_msgs::OccupancyGrid &occ_grid, int size, vector<bool> *selection=NULL)
{
  vector<nav_msgs::OccupancyGrid> snippets;
  
  cout << "Input Map Dimensions: " << occ_grid.info.height << " x " << occ_grid.info.width << " : " << occ_grid.data.size() << endl;
  
  for (int i = size / 2; i < occ_grid.info.height - size / 2; i++)
  {
    for (int j = size / 2; j < occ_grid.info.width - size / 2; j++)
    {
      int idx = i * occ_grid.info.width + j;
      
      int selIdx = i * (occ_grid.info.width - size) + j;
      
      if (selection != NULL && (*selection)[selIdx] == 0) continue;
      
      nav_msgs::OccupancyGrid snippet = occ_grid;
      snippet.info.height = size;
      snippet.info.width = size;
      
      geometry_msgs::Pose pos;
      pos.position.x = occ_grid.info.origin.position.x + j * occ_grid.info.resolution;
      pos.position.y = occ_grid.info.origin.position.y + i * occ_grid.info.resolution;
      snippet.info.origin.position = pos.position;
      
      Position gridPos(pos.position.x, pos.position.y);
      
      bool isSuccess;
      SubmapGeometry geom(*gridPtr, gridPos, Length(17 * gridPtr->getResolution(), 17 * gridPtr->getResolution()), isSuccess);
      
      if (isSuccess)
      {
	vector<int8_t> vec;
	for (SubmapIterator iter(geom); !iter.isPastEnd(); ++iter)
	{
	  vec.push_back((gridPtr->at("elevation", *iter) + 0.5) * 100);
	}
	
	snippet.data = vec;
      
	snippets.push_back(snippet);
      }
      else
      {
	cout << "Invalid submap" << endl;
      }
      
      /*vector<int8_t> vec;
      for (int k = 0; k < size; k++)
      {
	int basedx = idx + k * occ_grid.info.width;
	vec.insert(vec.end(), &(occ_grid.data[basedx]), &(occ_grid.data[basedx + size]));
      }
      snippet.data = vec;
      
      snippets.push_back(snippet);*/
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

configMap bloatMap(const vector<env_characterization::Feature> &data, int height, int width, const vector<Config> &configs, vector<float> angles, int nFeats, int reconRad)
{
  //Init config-space map
  configMap planGraph;
  
  for (int c = 0; c <= configs.size(); c++)
  {
    int angleSize = 1;
    if (c < configs.size() && !configs[c].omni) angleSize = angles.size();
    
    planGraph.push_back(
      thetaSubMap(angleSize, gridSubMap(height, vector<vector<bool>>(width, vector<bool>(nFeats + 1, 0)))));
  }
  
  //Iterate over cells
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      int feature = data[i * width + j].feature;
      int param = data[i * width + j].param;
      
      //if (feature == 0) continue;
      
      //Iterate over configs
      for (int c = 0; c < configs.size() + 1; c++)
      {
	if (c == configs.size())
	{
	  if (feature == 0) break;
	  
	  bloatRadius(planGraph[c][0], i, j, reconRad, 0);
	  break;
	}
	
	Config conf = configs[c];
	
	int bloatFeat = (feature > 0) + 1;
	if (conf.obs[feature > 0] || feature < 0) bloatFeat = 0;
	
	if (c == configs.size())
	{
	  bloatRadius(planGraph[c][0], i, j, conf.r, bloatFeat);
	}
	else if (conf.omni)
	{
	  bloatRadius(planGraph[c][0], i, j, conf.r, bloatFeat);
	}
	else
	{
	  for (int a = 0; a < angles.size(); a++)
	  {
	    bloatFoot(planGraph[c][a], i, j, conf.w, conf.h, angles[a] * 3.141592 / 180, bloatFeat);
	  }
	}
      }
    }
  }
  
  return planGraph;
}

void CreateGraph(configNodeMap &graphGrid, vector<Node> &graph, const configMap &bloatedMap,
		 const vector<env_characterization::Feature> &features, int height, int width,
		 vector<Config> configs, int nAngles, vector<vector<vector<Link>>> featLinks)
{
  for (int c = 0; c < configs.size(); c++)
  {
    graphGrid.push_back(
      thetaNodeMap((configs[c].omni ? 1 : nAngles), gridNodeMap(height, vector<int>(width, -1))));
  }
  
  int pointer = 0;
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {      
      for (int c = 0; c < configs.size(); c++)
      {
	for (int theta = 0; theta < (configs[c].omni ? 1 : nAngles); theta++)
	{
	  vector<bool> bloatFeats = bloatedMap[c][theta][i][j];
	  if (!bloatFeats[0])
	  {
	    graph.push_back(Node(Pose(i, j, theta), c));
	    graphGrid[c][theta][i][j] = pointer++;
	  }
	}
      }
    }
  }
  
  cout << "Sizes: " << graphGrid[1].size() << endl;
  
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      for (int c = 0; c < configs.size(); c++)
      {
	for (int theta = 0; theta < graphGrid[c].size(); theta++)
	{
	  int nodeIdx = graphGrid[c][theta][i][j];
	  
	  if (nodeIdx >= 0)
	  {
	    //Connect action neighbs
	    vector<Pose> actions = configs[c].getActions(theta);
	    
	    for (int a = 0; a < actions.size(); a++)
	    {
	      int y = i + actions[a].y;
	      int x = j + actions[a].x;
	      int th = actions[a].theta;
	      
	      if (th >= 0 && th < graphGrid[c].size() && y >= 0 && y < graphGrid[c][th].size() && x >= 0 && x < graphGrid[c][th][y].size())
	      {
		int neighbIdx = graphGrid[c][th][y][x];
		
		if (neighbIdx >= 0)
		{
		  graph[nodeIdx].neighbs.push_back(neighbIdx);
		  
		  float length = sqrt(pow(actions[a].x, 2) + pow(actions[a].y, 2));
		  
		  float maxCost = 0;
		  for (int f = 1; f < bloatedMap[c][theta][i][j].size(); f++)
		  {
		    if (bloatedMap[c][theta][i][j][f] || bloatedMap[c][th][y][x][f])
		    {
		      maxCost = max(maxCost, configs[c].costs[a][f - 1]);
		    }
		  }
		  
		  graph[nodeIdx].edgeCosts.push_back(maxCost * length);
		  graph[nodeIdx].actions.push_back(a);
		}
	      }
	      /*else
	      {
		graph[nodeIdx].neighbs.push_back(-1);
		graph[nodeIdx].edgeCosts.push_back(numeric_limits<float>::infinity());
	      }*/
	    }
	    
	    //If reconf, connect config neighbs
	    if (!bloatedMap[configs.size()][0][i][j][0])
	    {
	      for (int nC = 0; nC < configs.size(); nC++)
	      {
		if (nC == c) continue;
		
		for (int nTh = 0; nTh < graphGrid[nC].size(); nTh++)
		{
		  int neighbIdx = graphGrid[nC][nTh][i][j];
		  
		  if (neighbIdx >= 0)
		  {
		    graph[nodeIdx].neighbs.push_back(neighbIdx);
		    graph[nodeIdx].edgeCosts.push_back(50.0);
		    graph[nodeIdx].actions.push_back(-1);
		  }
		}
	      }
	    }
	  }
	}
      }
    }
  }
  
  //Now add in feature special action neighbs
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      int feat = features[i * width + j].feature;
      int param = features[i * width + j].param;
      
      /*if (feat > 0)
      {
	vector<Link> links = featLinks[feat][param];
	
	for (int l = 0; l < links.size(); l++)
	{
	  Link link = links[l];
	  int nodeIdx1 = graphGrid[link.vertices.first.c][link.vertices.first.th][i + link.vertices.first.i][j + link.vertices.first.j];
	  int nodeIdx2 = graphGrid[link.vertices.second.c][link.vertices.second.th][i + link.vertices.second.i][j + link.vertices.second.j];
	  
	  if (nodeIdx1 > -1 && nodeIdx2 > -1)
	  {
	    graph[nodeIdx1].neighbs.push_back(nodeIdx2);
	    graph[nodeIdx1].edgeCosts.push_back(link.cost);
	    graph[nodeIdx1].actions.push_back(link.action);
	  }
	}
      }*/
      
      /*if (feat == 1)
      {
	switch (int(params[0]))
	{
	  //Ledge feature at angle 6, corresponding to c = 1 (snake), th = 6, a = 2 (climb reverse on angle of 135 degrees)
	  case 5:
	    int nodeIdx1 = graphGrid[1][6][i + 6][j - 6];
	    int nodeIdx2 = graphGrid[1][6][i - 6][j + 6];
	    
	    if (nodeIdx1 > -1 && nodeIdx2 > -1 && features[(i + 6) * width + (j - 6)].feature == 0 &&
	      features[(i - 6) * width + (j + 6)].feature == 0)
	    {
	      graph[nodeIdx1].neighbs.push_back(nodeIdx2);
	      graph[nodeIdx1].edgeCosts.push_back(sqrt(pow(9.0, 2) * 2));
	      graph[nodeIdx1].actions.push_back(3);
	    }
	    break;
	}
      }*/
    }
  }
}

struct nodeCompare
{
  bool operator()(Node* lhs, Node* rhs)
  {
    return lhs->cost > rhs->cost;
  }
};

void planPath(vector<Node> &nodeGraph, Node* start, Node* goal=NULL)
{
  priority_queue<Node*, vector<Node*>, nodeCompare> pq;
  
  start->cost = 0;
  pq.push(start);
  
  while (!pq.empty())
  {
    Node* minNode = pq.top();
    pq.pop();
    
    for (int i = 0; i < minNode->neighbs.size(); i++)
    {
      int neighbIdx = minNode->neighbs[i];
      if (neighbIdx > 0)
      {
	Node* neighb = &nodeGraph[minNode->neighbs[i]];
	if (minNode->cost + minNode->edgeCosts[i] < neighb->cost)
	{
	  neighb->cost = minNode->cost + minNode->edgeCosts[i];
	  neighb->parent = minNode;
	  neighb->action = minNode->actions[i];
	  
	  pq.push(neighb);
	}
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "env_characterization");
  ros::NodeHandle node;
  
  ros::Rate rate(2.0);
  
  ros::Subscriber grid_sub = node.subscribe("/debug_grid", 5, grid_cb);
  ros::Subscriber cloud_sub = node.subscribe("/camera/depth_registered/points", 5, cloud_cb);
  ros::Subscriber click_sub = node.subscribe("/initialpose", 1, click_cb);
  
  ros::Publisher grid_pub = node.advertise<grid_map_msgs::GridMap>("/elevation", 1);
  ros::Publisher sub_pub = node.advertise<grid_map_msgs::GridMap>("/elevation_sub", 1);
  ros::Publisher occ_pub = node.advertise<nav_msgs::OccupancyGrid>("/input_occ_map", 5);
  ros::Publisher class_pub = node.advertise<nav_msgs::OccupancyGrid>("/classes", 5);
  ros::Publisher bloat_pub = node.advertise<nav_msgs::OccupancyGrid>("/bloated_map", 5);
  ros::Publisher flat_pub = node.advertise<nav_msgs::OccupancyGrid>("/flat_map", 5);
  ros::Publisher cost_pub = node.advertise<nav_msgs::OccupancyGrid>("/cost_map", 5);
  snip_pub = node.advertise<grid_map_msgs::GridMap>("/snip", 1);
  
  ros::Publisher config0_pub = node.advertise<nav_msgs::OccupancyGrid>("/config0", 5);
  ros::Publisher config10_pub = node.advertise<nav_msgs::OccupancyGrid>("/config1_0", 5);
  ros::Publisher config11_pub = node.advertise<nav_msgs::OccupancyGrid>("/config1_1", 5);
  ros::Publisher config12_pub = node.advertise<nav_msgs::OccupancyGrid>("/config1_2", 5);
  ros::Publisher config13_pub = node.advertise<nav_msgs::OccupancyGrid>("/config1_3", 5);
  
  ros::Publisher edge0_pub = node.advertise<visualization_msgs::Marker>("/edges0", 5);
  ros::Publisher edge10_pub = node.advertise<visualization_msgs::Marker>("/edges1_0", 5);
  ros::Publisher edge11_pub = node.advertise<visualization_msgs::Marker>("/edges1_1", 5);
  ros::Publisher edge12_pub = node.advertise<visualization_msgs::Marker>("/edges1_2", 5);
  ros::Publisher edge13_pub = node.advertise<visualization_msgs::Marker>("/edges1_3", 5);
  ros::Publisher edgeC_pub = node.advertise<visualization_msgs::Marker>("/edgesC", 5);
  ros::Publisher path_viz_pub = node.advertise<visualization_msgs::Marker>("/path_viz", 5);
  ros::Publisher path_pub = node.advertise<env_characterization::PathNodeArray>("/path", 5);
  
  ros::ServiceClient client = node.serviceClient<env_characterization::classify_map>("/classify_map");
  clientPtr = &client;
  
  listener = new tf::TransformListener;
  
  GridMap grid({"elevation", "feature", "param", "bloat", "cost"});
  grid.setFrameId("map");
  
  /*if (debug)
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
  tree.insertPointCloud(octomap_cloud, octomap::point3d(0, 0, 0), sensorPose, -1, false, true);*/
  
  /*octomap::OcTree tree("/home/jonathan/catkin_ws/smore_map3.bt");
  
  octomap::OcTree tree2(0.02);
  
  for (octomap::OcTree::leaf_iterator iter = tree.begin_leafs(15); iter != tree.end_leafs(); iter++)
  {
    //octomap::OcTreeNode node = *iter;
    if (tree.isNodeOccupied(*iter))
    {
      tree2.setNodeValue(iter.getCoordinate(), iter->getLogOdds());
    }
  }
  
  GridMapOctomapConverter::fromOctomap(tree, "elevation", grid);*/
  
  string img_path = "/home/jonathan/catkin_ws/env_2_2.png";
  cv::Mat originalImage = cv::imread(img_path);  
  //GridMapCvConverter::initializeFromImage(originalImage, 0.01, grid, Position(0.0, 0.0));
  
  grid.setGeometry(Length(2.57, 2.57), 0.01, Position(0.0, 0.0));
  GridMapCvConverter::addLayerFromImage<uint8_t, 1>(originalImage, "elevation", grid);
  //grid.setPosition(Position(0.0, 0.0));
  gridPtr = &grid;
  
  GridMapRosConverter::toMessage(grid, grid_msg);
  grid_pub.publish(grid_msg);
  
  cout << "Map Position: " << grid.getPosition()[0] << ", " << grid.getPosition()[1] << endl;
  
  //Save Depth Image
  /*float maxVal = 0;
  for (GridMapIterator iter(grid); !iter.isPastEnd(); ++iter)
  {
    maxVal = max(maxVal, grid.at("elevation", iter.getUnwrappedIndex()));
  }
  cout << "Maximum Height: " << maxVal << endl;
  
  cv::Mat originalImage;
  GridMapCvConverter::toImage<uint8_t, 1>(grid, "elevation", CV_8UC1, 0.0, 1.0, originalImage);
  string img_file = "/home/jonathan/catkin_ws/elevation_map.png";
  cv::imwrite(img_file, originalImage);*/
  
  //GridMapCvConverter::addLayerFromImage<uint8_t, 1>(originalImage, "elevation", grid);
  
  chrono::steady_clock::time_point totStart = chrono::steady_clock::now();
  
  //Find flat regions in map
  double mapCush = 0.2;
  cout << "Map Size: " << grid.getSize()[0] << " x " << grid.getSize()[1] << endl;
  vector<env_characterization::Feature> features(grid.getSize()[0] * grid.getSize()[1], env_characterization::Feature());
  for (vector<env_characterization::Feature>::iterator iter = features.begin(); iter != features.end(); iter++)
  {
    iter->feature = -1;
  }
  
  for (GridMapIterator iter(grid); !iter.isPastEnd(); ++iter)
  {
    grid.at("feature", *iter) = -1;
    grid.at("param", *iter) = 0;
    grid.at("bloat", *iter) = 0;
    grid.at("cost", *iter) = 0;
  }
  
  cout << "Feature map length: " << features.size() << endl;
  
  Flattenize(grid, &features, mapCush, 3, 0.0001);
  
  vector<std_msgs::Float32MultiArray> partitions = Snippetize(grid, mapCush);
  
  cout << "Partitions: " << partitions.size() << endl;
  
  env_characterization::classify_map srv;
  srv.request.partitions = partitions;
  
  vector<env_characterization::Feature> featureMap;
  
  chrono::steady_clock::time_point start = chrono::steady_clock::now();
  
  chrono::steady_clock::time_point t1;
  
  if (client.call(srv))
  {
    cout << "Service Responded" << endl;
    
    t1 = chrono::steady_clock::now();

    cout << "Classification Time: " << chrono::duration_cast<chrono::microseconds>(t1 - start).count() / 1000000.0 << endl;
    
    int featCt = 0;
    for (SubmapIterator iter(grid, Index(10, 10), grid.getSize() - Size(20, 20)); !iter.isPastEnd(); ++iter)
    {
      if (int(grid.at("feature", *iter)) == -1)
      {
	grid.at("feature", *iter) = srv.response.characters[featCt].feature;
	grid.at("param", *iter) = srv.response.characters[featCt].param;
	featCt++;
      }
      
      env_characterization::Feature feat;
      feat.feature = grid.at("feature", *iter);
      feat.param = grid.at("param", *iter);
      featureMap.push_back(feat);
    }
    
    cout << "featCt: " << featCt << endl;
    
    /*GridMapRosConverter::toMessage(grid, grid_msg);
    while (ros::ok())
    {
      grid_pub.publish(grid_msg);
      rate.sleep();
      ros::spinOnce();
    }*/
  }
  else
  {
    cout << "Service Failed" << endl;
    
    while (ros::ok())
    {
      grid_pub.publish(grid_msg);
      snip_pub.publish(snip_msg);
      
      ros::spinOnce();
      rate.sleep();
    }
  }
  
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  
  cout << "Flattenize Time: " << chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000000.0 << endl;
  
  //Planning Graph Construction
  //Obstacle candidate list: [flat, ledge]
  vector<float> flatCosts = {1.0, numeric_limits<float>::infinity()};
  Config carCon(true, {0, 1}, vector<vector<float>>(8, flatCosts), 11, 5, 5);
  Config snakeCon(false, {0, 1}, {flatCosts, flatCosts, {3.0, 3.0}, {3.0, 3.0}}, 11, 5, 7);
  vector<Config> configList = {carCon, snakeCon};
  
  vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435};
  
  //Feature Actions
  
  //Vectors below represent:
  //feat->param->link
  vector<vector<vector<Link>>> featLinks;
  vector<vector<Link>> ledgeLinks;
  
  ledgeLinks.push_back({Link(make_pair(NodeSpec(1, 0, 0, 12), NodeSpec(1, 0, 0, -12)), 4, 24 * 3)});
  featLinks.push_back(ledgeLinks);
  
  //Translate featureMap to bloatFeats
  vector<int> bloatFeats;
  
  int graphHeight = grid.getSize()[0] - 20;
  int graphWidth = grid.getSize()[1] - 20;
  
  int reconRad = 12;
  configMap planGraph = bloatMap(featureMap, graphHeight, graphWidth, configList, angles, 2, reconRad);
  
  for (int i = 0; i < planGraph[0][0].size(); i++)
  {
    for (int j = 0; j < planGraph[0][0][i].size(); j++)
    {
      if (planGraph[0][0][i][j][0])
      {
	grid.at("bloat", Index(i, j) + Index(10, 10)) = 1;
      }
    }
  }
  
  cout << "Graph Sizes: car angles " << planGraph[0].size() << "; snake angles " << planGraph[1].size() << endl;
  
  cout << "Starting Node Graph Generation" << endl;
  
  vector<Node> nodeGraph;
  configNodeMap nodeGrid;
  
  CreateGraph(nodeGrid, nodeGraph, planGraph, featureMap, graphHeight, graphWidth, configList, angles.size(), featLinks);
  
  cout << "Finished Node Graph Creation" << endl;
  cout << "Graph Size: " << nodeGraph.size() << endl;
  
  chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
  
  cout << "Graph Creation Time: " << chrono::duration_cast<chrono::microseconds>(t3 - t2).count() / 1000000.0 << endl;
  
  cout << "Begin Dijkstra Path Planning" << endl;
  
  //Dijkstra Path Planning
  //Node* startNode = &nodeGraph[nodeGrid[0][0][30][10]];
  cout << "Start Node: " << nodeGrid[0][0][100][200] << endl;
  Node* startNode = &nodeGraph[nodeGrid[0][0][100][200]];
  planPath(nodeGraph, startNode);
  
  cout << "Finished Path Planning" << endl;
  
  chrono::steady_clock::time_point totStop = chrono::steady_clock::now();
  cout << "Path Planning Time: " << chrono::duration_cast<chrono::microseconds>(totStop - t3).count() / 1000000.0 << endl;
  cout << "Total Processing Time: " << chrono::duration_cast<chrono::microseconds>(totStop - totStart).count() / 1000000.0 << endl;
  
  float res = grid.getResolution();
  
  cout << "Resolution: " << res << endl;
  
  //Publish path msg
  env_characterization::PathNodeArray path;
  
  vector<env_characterization::PathNode> rev_path;
  
  Node* goalNode = &nodeGraph[nodeGrid[1][6][100][200]];
  //Node* goalNode = &nodeGraph[nodeGrid[0][0][10 + 28][83]];
  Node* curParent = goalNode->parent;
  
  env_characterization::PathNode pathNode;
  pathNode.config = goalNode->config;
  pathNode.theta = angles[goalNode->pose.theta] * 3.141592 / 180;
  pathNode.action = goalNode->action;
  
  geometry_msgs::Pose waypt;
  waypt.position.x = goalNode->pose.x * res + grid.getPosition()[0] + res / 2;
  waypt.position.y = goalNode->pose.y * res + grid.getPosition()[1] + res / 2;
  waypt.position.z = 0;
  pathNode.pose = waypt;
  
  path.path.push_back(pathNode);
  
  cout << goalNode->pose.x << ", " << goalNode->pose.y << " : " << goalNode->config << endl;
  cout << "Cost: " << goalNode->cost << endl;
  while (curParent != NULL)
  {
    //cout << curParent->pose.x << ", " << curParent->pose.y << " : " << curParent->config << endl;
    
    pathNode.config = curParent->config;
    pathNode.theta = angles[curParent->pose.theta] * 3.141592 / 180;
    pathNode.action = curParent->action;
    
    waypt.position.x = curParent->pose.x * res + grid.getPosition()[0] + res / 2;
    waypt.position.y = curParent->pose.y * res + grid.getPosition()[1] + res / 2;
    waypt.position.z = 0;
    pathNode.pose = waypt;
    
    path.path.push_back(pathNode);
    
    curParent = curParent->parent;
  }
  
  reverse(path.path.begin(), path.path.end());
  
  /*for (int i = 0; i < path.path.size(); i++)
  {
    if (path.path[i].config == 1)
    {
      path.path[i + 10].action = 3;
      path.path.erase(path.path.begin() + i + 11, path.path.end());
      break;
    }
  }*/
  
  //Visualization Stuff
  
  cout << "Visualizing Stuff" << endl;
  
  cout << graphWidth << " : " << graphHeight << endl;
  
  //Visualize path cost
  float maxCost = 0;
  for (int i = 0; i < graphHeight; i++)
  {
    for (int j = 0; j < graphWidth; j++)
    {
      float minCost = numeric_limits<float>::infinity();
      for (int c = 0; c < configList.size(); c++)
      {
	for (int theta = 0; theta < nodeGrid[c].size(); theta++)
	{
	  int idx = nodeGrid[c][theta][i][j];
	  if (idx >= 0) minCost = min(nodeGraph[idx].cost, minCost);
	}
      }
      
      if (minCost < numeric_limits<float>::infinity())
      {
	grid.at("cost", Index(i, j) + Index(10, 10)) = minCost;
	maxCost = max(maxCost, minCost);
      }
      else
      {
	grid.at("cost", Index(i, j) + Index(10, 10)) = -1;
      }
    }
  }
  
  cout << "Visualize Graph" << endl;
  
  bool isSuccess;
  GridMap subGrid = grid.getSubmap(grid.getPosition(), Length(graphWidth, graphHeight) * grid.getResolution(), isSuccess);
  
  nav_msgs::OccupancyGrid occ_grid;
  nav_msgs::OccupancyGrid classes;
  nav_msgs::OccupancyGrid bloat_msg;
  nav_msgs::OccupancyGrid costMap;
  GridMapRosConverter::toOccupancyGrid(subGrid, "elevation", -0.5, 0.5, occ_grid);
  GridMapRosConverter::toOccupancyGrid(subGrid, "feature", -0.5, 0.5, classes);
  GridMapRosConverter::toOccupancyGrid(subGrid, "bloat", -0.5, 0.5, bloat_msg);
  GridMapRosConverter::toOccupancyGrid(subGrid, "cost", -1, maxCost + 10, costMap);
  
  //Visualize Planning Graph
  visualization_msgs::Marker lineTemp;
  lineTemp.header.frame_id = occ_grid.header.frame_id;
  lineTemp.header.stamp = occ_grid.header.stamp;
  lineTemp.ns = "graph_edges";
  lineTemp.action = visualization_msgs::Marker::ADD;
  
  lineTemp.pose.orientation.w = 1.0;
  
  lineTemp.id = 0;
  lineTemp.type = visualization_msgs::Marker::LINE_LIST;
  
  lineTemp.scale.x = 0.0006;
  lineTemp.color.r = 1.0;
  lineTemp.color.a = 1.0;
  
  visualization_msgs::Marker lineC = lineTemp;
  lineC.color.g = 1.0;
  vector<visualization_msgs::Marker> linez;
  
  nav_msgs::OccupancyGrid vizTemplate = occ_grid;
  
  for (int i = 0; i < vizTemplate.data.size(); i++)
  {
    vizTemplate.data[i] = 0;
  }
  
  vector<nav_msgs::OccupancyGrid> vizMaps;
  
  for (int c = 0; c < configList.size(); c++)
  {
    for (int theta = 3 * !configList[c].omni; theta < min(7, int(nodeGrid[c].size())); theta++)
    {
      vizMaps.push_back(vizTemplate);
      vizMaps.back().info.origin.position.z = (c + 1) * 0.2 + theta * 0.05;
      
      lineTemp.id = linez.size() + 1;
      lineTemp.color.r = (c == 0) * 1.0;
      lineTemp.color.g = (c == 1) * 1.0;
      lineTemp.color.b = (c == 2) * 1.0;
      
      linez.push_back(lineTemp);
      
      for (int i = 40; i < 70; i++)
      {
	for (int j = 30; j < 70; j++)
	{
	  //cout << c << ", " << theta << endl;
	  int nodeIdx = nodeGrid[c][theta][i][j];
	  if (nodeIdx >= 0)
	  {
	    (vizMaps.back()).data[i * occ_grid.info.width + j] = 10;
	    
	    geometry_msgs::Point p1;
	    p1.x = nodeGraph[nodeIdx].pose.y * res + occ_grid.info.origin.position.x + res / 2;
	    p1.y = nodeGraph[nodeIdx].pose.x * res + occ_grid.info.origin.position.y + res / 2;
	    p1.z = (c + 1) * 0.2 + theta * 0.05;
	    
	    vector<int> neighbs = nodeGraph[nodeIdx].neighbs;
	    for (int n = 0; n < neighbs.size(); n++)
	    {
	      geometry_msgs::Point p2;
	      p2.x = nodeGraph[neighbs[n]].pose.y * res + occ_grid.info.origin.position.x + res / 2;
	      p2.y = nodeGraph[neighbs[n]].pose.x * res + occ_grid.info.origin.position.y + res / 2;
	      p2.z = (nodeGraph[neighbs[n]].config + 1) * 0.2 + nodeGraph[neighbs[n]].pose.theta * 0.05;
	      
	      if (nodeGraph[neighbs[n]].config != c)
	      {
		if (nodeGraph[neighbs[n]].pose.theta >= 2 && nodeGraph[neighbs[n]].pose.theta < 6)
		{
		  lineC.points.push_back(p1);
		  lineC.points.push_back(p2);
		}
	      }
	      else
	      {
		linez.back().points.push_back(p1);
		linez.back().points.push_back(p2);
	      }
	    }
	  }
	}
      }
    }
  }
  
  //Visualize Path
  visualization_msgs::Marker pathLine = lineTemp;
  pathLine.id = linez.size() + 2;
  pathLine.color.r = 1.0;
  pathLine.color.g = 1.0;
  pathLine.color.b = 1.0;
  pathLine.scale.x = 0.001;
  
  cout << occ_grid.info.origin.position.y << endl;
  curParent = goalNode->parent;
  
  geometry_msgs::Point p;
  p.x = goalNode->pose.y * res + occ_grid.info.origin.position.x + res / 2;
  p.y = goalNode->pose.x * res + occ_grid.info.origin.position.y + res / 2;
  p.z = (goalNode->config) * 0.02 + goalNode->pose.theta * 0.01;
  
  cout << goalNode->pose.x << ", " << goalNode->pose.y << " : " << goalNode->config << endl;
  cout << "Cost: " << goalNode->cost << endl;
  while (curParent != NULL)
  {
    //cout << curParent->pose.x << ", " << curParent->pose.y << " : " << curParent->config << endl;
    
    pathLine.points.push_back(p);
    p.x = curParent->pose.y * res + occ_grid.info.origin.position.x + res / 2;
    p.y = curParent->pose.x * res + occ_grid.info.origin.position.y + res / 2;
    p.z = (curParent->config) * 0.02 + curParent->pose.theta * 0.01;
    pathLine.points.push_back(p);
    
    curParent = curParent->parent;
  }
  
  /*goalNode = &nodeGraph[nodeGrid[0][0][10 + 28][83]];
  curParent = goalNode->parent;
  
  p.x = goalNode->pose.y * res + occ_grid.info.origin.position.x + res / 2;
  p.y = goalNode->pose.x * res + occ_grid.info.origin.position.y + res / 2;
  p.z = (goalNode->config) * 0.02 + goalNode->pose.theta * 0.01;
  
  cout << goalNode->pose.x << ", " << goalNode->pose.y << " : " << goalNode->config << endl;
  cout << "Cost: " << goalNode->cost << endl;
  while (curParent != NULL)
  {
    //cout << curParent->pose.x << ", " << curParent->pose.y << " : " << curParent->config << endl;
    
    pathLine.points.push_back(p);
    p.x = curParent->pose.y * res + occ_grid.info.origin.position.x + res / 2;
    p.y = curParent->pose.x * res + occ_grid.info.origin.position.y + res / 2;
    p.z = (curParent->config) * 0.02 + curParent->pose.theta * 0.01;
    pathLine.points.push_back(p);
    
    curParent = curParent->parent;
  }*/
  
  GridMapRosConverter::toMessage(grid, grid_msg);
  
  while (ros::ok())
  {
    occ_pub.publish(occ_grid);
    class_pub.publish(classes);
    grid_pub.publish(grid_msg);
    bloat_pub.publish(bloat_msg);
    cost_pub.publish(costMap);
    snip_pub.publish(snip_msg);
    
    config0_pub.publish(vizMaps[0]);
    config10_pub.publish(vizMaps[1]);
    config11_pub.publish(vizMaps[2]);
    config12_pub.publish(vizMaps[3]);
    config13_pub.publish(vizMaps[4]);
    
    edge0_pub.publish(linez[0]);
    edge10_pub.publish(linez[1]);
    edge11_pub.publish(linez[2]);
    edge12_pub.publish(linez[3]);
    edge13_pub.publish(linez[4]);
    edgeC_pub.publish(lineC);
    
    path_viz_pub.publish(pathLine);
    path_pub.publish(path);
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}