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
#include <env_characterization/path_srv.h>

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
visualization_msgs::Marker* pathVizPtr;
env_characterization::PathNodeArray* pathPtr;

ros::ServiceClient* clientPtr;
ros::ServiceClient* path_clientPtr;

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

//vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435};
vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435,
      180, 180 + 26.5651, 180 + 45, 180 + 63.4349, 180 + 90, 180 + 116.565, 180 + 135, 180 + 153.435};

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
  pair<vector<bool>, vector<bool>> obs;
  int action;
  float cost;
  
  Link(pair<NodeSpec, NodeSpec> v, pair<vector<bool>, vector<bool>> o, int a, float c)
  {
    vertices = v;
    obs = o;
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
  int w;			//Width of robot footprint
  int h;			//Height of robot footprint
  int r;			//Radius of robot footprint (omni)
  
  Config(bool omni, int h, int w, int r)
  {
    this->omni = omni;
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

vector<Node>* nodeGraphPtr;
configNodeMap* nodeGridPtr;

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
      
      if (feature == 0) continue;
      
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
	
	int bloatFeat = feature + 1;
	//if (conf.obs[feature > 0] || feature < 0) bloatFeat = 0;
	
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

bool validNode(vector<bool> v1, vector<bool> v2)
{
  if (v1.size() != v2.size())
  {
    cout << "Mismatched sizes" << endl;
    return false;
  }
  
  for (int i = 0; i < v1.size(); i++)
  {
    if (v1[i] && v2[i]) return false;
  }
  
  return true;
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
	  graph.push_back(Node(Pose(i, j, theta), c));
	  graphGrid[c][theta][i][j] = pointer++;
	}
      }
    }
  }
  
  cout << "Sizes: " << graphGrid[1].size() << endl;
  
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      int feat = features[i * width + j].feature;
      int param = features[i * width + j].param;
      
      if (feat >= 0)
      {
	vector<Link> linkz = featLinks[feat][param];
	
	for (int l = 0; l < linkz.size(); l++)
	{
	  NodeSpec n1 = linkz[l].vertices.first;
	  NodeSpec n2 = linkz[l].vertices.second;
	  
	  if (n1.th >= 0 && n1.th < graphGrid[n1.c].size() && i + n1.i >= 0 && i + n1.i < graphGrid[n1.c][n1.th].size() && j + n1.j >= 0 && j + n1.j < graphGrid[n1.c][n1.th][i + n1.i].size()
	    && n2.th >= 0 && n2.th < graphGrid[n2.c].size() && i + n2.i >= 0 && i + n2.i < graphGrid[n2.c][n2.th].size() && j + n2.j >= 0 && j + n2.j < graphGrid[n2.c][n2.th][i + n2.i].size())
	  {
	    int nodeIdx1 = graphGrid[n1.c][n1.th][i + n1.i][j + n1.j];
	    int nodeIdx2 = graphGrid[n2.c][n2.th][i + n2.i][j + n2.j];
	    
	    vector<bool> obs1 = linkz[l].obs.first;
	    vector<bool> obs2 = bloatedMap[n1.c][n1.th][i + n1.i][j + n1.j];
	    
	    /*if (feat == 2)
	    {
	      cout << "Link Obs" << endl;
	      for (int i = 0; i < obs1.size(); i++)
	      {
		cout << obs1[i] << endl;
	      }
	      cout << "Bloat Obs" << endl;
	      for (int i = 0; i < obs2.size(); i++)
	      {
		cout << obs2[i] << endl;
	      }
	    }*/
	    
	    if (validNode(linkz[l].obs.first, bloatedMap[n1.c][n1.th][i + n1.i][j + n1.j])
	      && validNode(linkz[l].obs.second, bloatedMap[n2.c][n2.th][i + n2.i][j + n2.j]))
	    {
	      graph[nodeIdx1].neighbs.push_back(nodeIdx2);
	      graph[nodeIdx1].edgeCosts.push_back(linkz[l].cost * sqrt(pow(n2.i - n1.i, 2) + pow(n2.j - n1.j, 2)));
	      graph[nodeIdx1].actions.push_back(linkz[l].action);
	    }
	  }
	}
      }
      
      //If reconf, connect config neighbs
      if (!bloatedMap[configs.size()][0][i][j][0])
      {
	for (int c = 0; c < configs.size(); c++)
	{
	  for (int theta = 0; theta < graphGrid[c].size(); theta++)
	  {
	    int nodeIdx = graphGrid[c][theta][i][j];

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

struct nodeCompare
{
  bool operator()(Node* lhs, Node* rhs)
  {
    return lhs->cost > rhs->cost;
  }
};

void planPath(vector<Node> &nodeGraph, Node* start, Node* goal=NULL)
{
  chrono::steady_clock::time_point t0 = chrono::steady_clock::now();
  
  priority_queue<Node*, vector<Node*>, nodeCompare> pq;
  
  for (int i = 0; i < nodeGraph.size(); i++)
  {
    nodeGraph[i].parent = NULL;
    nodeGraph[i].cost = numeric_limits<float>::infinity();
  }
  
  start->cost = 0;
  pq.push(start);
  
  while (!pq.empty())
  {
    Node* minNode = pq.top();
    pq.pop();
    
    if (goal != NULL && minNode->config == goal->config && minNode->pose.x == goal->pose.x && minNode->pose.y == goal->pose.y
      && minNode->pose.theta == goal->pose.theta)
    {
      break;
    }
    
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
  
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  cout << "Path Planning Time: " << chrono::duration_cast<chrono::microseconds>(t1 - t0).count() / 1000000.0 << endl;
}

Config carCon(true, 0, 0, 15);
Config snakeCon(false, 23, 7, 0);
Config dolphinCon(false, 23, 12, 0);
vector<Config> configList = {carCon, snakeCon, dolphinCon};
int reconRad = 35;

bool path_cb(env_characterization::path_srv::Request &req, env_characterization::path_srv::Response &res)
{
  cout << "Acquiring Path" << endl;
  
  geometry_msgs::Pose startPose = req.start.pose;
  geometry_msgs::Pose goalPose = req.goal.pose;
  
  int c = req.start.config;
  double startYaw = req.start.theta * 180 / M_PI;
  if (startYaw < 0) startYaw += 360;
  
  vector<double> errors;
  for (int i = 0; i < angles.size(); i++)
  {
    errors.push_back(min(abs(startYaw - angles[i]), 360 - abs(startYaw - angles[i])));
  }
  int thID = distance(errors.begin(), min_element(errors.begin(), errors.end()));
  
  if (configList[c].omni) thID = 0;
  
  cout << "c, startYaw, thID: " << c << ", " << startYaw << ", " << thID << endl;
  
  double yaw = tf::getYaw(goalPose.orientation);
  
  Position zeroPos;
  gridPtr->getPosition(Index(10, 10), zeroPos);
  
  double gridRes = gridPtr->getResolution();
  
  Index idxStart;
  Index idxGoal;
  gridPtr->getIndex(Position(startPose.position.x, startPose.position.y), idxStart);
  gridPtr->getIndex(Position(goalPose.position.x, goalPose.position.y), idxGoal);
  
  idxStart -= Index(10, 10);
  idxGoal -= Index(10, 10);
  
  int x0 = idxStart[0];
  int y0 = idxStart[1];
  
  int xf = idxGoal[0];
  int yf = idxGoal[1];
  
  cout << x0 << " . " << y0 << " . " << xf << " . " << yf << endl;
  
  cout << "Start Node: " << (*nodeGridPtr)[c][thID][x0][y0] << endl;
  Node* startNode = &(*nodeGraphPtr)[(*nodeGridPtr)[c][thID][x0][y0]];
  Node* goalNode = &((*nodeGraphPtr)[(*nodeGridPtr)[0][0][xf][yf]]);
  planPath(*nodeGraphPtr, startNode);
  
  cout << "Finished Path Planning" << endl;
  
  float minCost = numeric_limits<float>::infinity();
  for (int c = 0; c < nodeGridPtr->size(); c++)
  {
    for (int th = 0; th < (*nodeGridPtr)[c].size(); th++)
    {
      Node* testNode = &(*nodeGraphPtr)[(*nodeGridPtr)[c][th][xf][yf]];
      cout << c << ", " << th << ": " << testNode->cost << endl;
      if (testNode->cost < minCost)
      {
	minCost = testNode->cost;
	goalNode = testNode;
      }
    }
  }
  Node* curParent = goalNode->parent;
  
  env_characterization::PathNodeArray path;
  env_characterization::PathNode pathNode;
  pathNode.config = goalNode->config;
  pathNode.theta = angles[goalNode->pose.theta] * 3.141592 / 180;
  pathNode.action = goalNode->action;
  
  geometry_msgs::Pose waypt;
  waypt.position.x = zeroPos[0] - goalNode->pose.x * gridRes + gridRes / 2;
  waypt.position.y = zeroPos[1] - goalNode->pose.y * gridRes + gridRes / 2;
  waypt.position.z = 0;
  pathNode.pose = waypt;
  
  path.path.push_back(pathNode);
  
  cout << goalNode->pose.x << ", " << goalNode->pose.y << " : " << goalNode->config << endl;
  cout << "Cost: " << goalNode->cost << endl;
  
  if (goalNode->cost == numeric_limits<float>::infinity()) return false;
  
  while (curParent != NULL)
  {
    //cout << curParent->pose.x << ", " << curParent->pose.y << " : " << curParent->config << endl;
    
    pathNode.config = curParent->config;
    pathNode.theta = angles[curParent->pose.theta] * 3.141592 / 180;
    pathNode.action = curParent->action;
    
    waypt.position.x = zeroPos[0] - curParent->pose.x * gridRes + gridRes / 2;
    waypt.position.y = zeroPos[1] - curParent->pose.y * gridRes + gridRes / 2;
    waypt.position.z = 0;
    pathNode.pose = waypt;
    
    path.path.push_back(pathNode);
    
    curParent = curParent->parent;
  }
  
  //reverse(path.path.begin(), path.path.end());
  
  res.path = path;
  
  *pathPtr = path;
  
  cout << "Path Extracted" << endl;
  
  //Visualize Path
  pathVizPtr->points.clear();
  
  for (int i = 0; i < path.path.size() - 1; i++)
  {
    geometry_msgs::Point p;
    
    env_characterization::PathNode node = path.path[i];
    p.x = node.pose.position.x;
    p.y = node.pose.position.y;
    p.z = node.config * 0.2 + 0.01;
    pathVizPtr->points.push_back(p);
    
    node = path.path[i + 1];
    p.x = node.pose.position.x;
    p.y = node.pose.position.y;
    p.z = node.config * 0.2 + 0.01;
    pathVizPtr->points.push_back(p);
  }
}

void goal_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  cout << "Nav Goal Received" << endl;
  geometry_msgs::Pose goalPose = msg->pose.pose;
  
  try
  {
    tf::StampedTransform egoTF;
    listener->lookupTransform("map", "nest", ros::Time(), egoTF);
    
    geometry_msgs::Pose startPose;
    startPose.position.x = egoTF.getOrigin().getX();
    startPose.position.y = egoTF.getOrigin().getY();
    startPose.orientation.w = 1;
    
    env_characterization::path_srv pathSrv;
    pathSrv.request.start.pose = startPose;
    pathSrv.request.goal.pose = goalPose;
    if (path_cb(pathSrv.request, pathSrv.response))
    {
      cout << "Got path" << endl;
    }
    else
    {
      cout << "Failed to get path" << endl;
    }
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("Couldn't get TF for yaw");
    
    geometry_msgs::Pose startPose;
    startPose.position.x = 0.5;
    startPose.position.y = -0.5;
    startPose.orientation.w = 1;
    
    env_characterization::path_srv pathSrv;
    pathSrv.request.start.pose = startPose;
    pathSrv.request.goal.pose = goalPose;
    if (path_cb(pathSrv.request, pathSrv.response))
    {
      cout << "Got path" << endl;
    }
    else
    {
      cout << "Failed to get path" << endl;
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
  ros::Subscriber click_sub = node.subscribe("/initialpose", 1, goal_cb);
  //ros::Subscriber goal_sub = node.subscribe("/move_base_simple/goal", 1, goal_cb);
  
  ros::Publisher grid_pub = node.advertise<grid_map_msgs::GridMap>("/elevation", 1);
  ros::Publisher sub_pub = node.advertise<grid_map_msgs::GridMap>("/elevation_sub", 1);
  ros::Publisher occ_pub = node.advertise<nav_msgs::OccupancyGrid>("/input_occ_map", 5);
  ros::Publisher class_pub = node.advertise<nav_msgs::OccupancyGrid>("/classes", 5);
  ros::Publisher bloat_pub = node.advertise<nav_msgs::OccupancyGrid>("/bloated_map", 5);
  ros::Publisher flat_pub = node.advertise<nav_msgs::OccupancyGrid>("/flat_map", 5);
  ros::Publisher cost_pub = node.advertise<nav_msgs::OccupancyGrid>("/cost_map", 5);
  snip_pub = node.advertise<grid_map_msgs::GridMap>("/snip", 1);
  
  ros::ServiceServer path_srv = node.advertiseService("/get_path", path_cb);
  
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
  
  chrono::steady_clock::time_point totStart = chrono::steady_clock::now();
  
  //Feature Actions
  
  //Vectors below represent:
  //feat->param->link
  vector<vector<vector<Link>>> featLinks;
  
  //Create action links for flat
  vector<Link> flatLinks;
  vector<Link> wavyLinks;
  
  //Create and push car links
  Link carLink(make_pair(NodeSpec(0, 0, 0, 0), NodeSpec(0, 0, 0, 0)), make_pair(vector<bool>({1, 0, 1, 1}), vector<bool>({1, 0, 1, 1})), 0, 1);
  vector<Link> carLinks;
  for (int i = -1; i <= 1; i ++)
  {
    for (int j = -1; j <= 1; j++)
    {
      if (abs(i) + abs(j) == 0) continue;
      
      Link l = carLink;
      l.vertices.second.i = i;
      l.vertices.second.j = j;
      
      flatLinks.push_back(l);
    }
  }
  
  //Create and push snake links
  Link snakeFlatLink(make_pair(NodeSpec(1, 0, 0, 0), NodeSpec(1, 0, 0, 0)),
		     make_pair(vector<bool>({1, 0, 1, 1}), vector<bool>({1, 0, 1, 1})), 0, 1);
  for (int theta = 0; theta < 16; theta++)
  {
    for (int i = -1; i <= 1; i += 2)
    {
      Link l = snakeFlatLink;
      l.vertices.first.th = theta;
      l.vertices.second.th = theta;
      
      //Forward-Backward Neighbors
      switch (theta % 8)
      {
	case 0:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 0;
	  break;
	case 1:
	  l.vertices.second.i = i * 2;
	  l.vertices.second.j = i * 1;
	  break;
	case 2:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 1;
	  break;
	case 3:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 2;
	  break;
	case 4:
	  l.vertices.second.i = i * 0;
	  l.vertices.second.j = i * 1;
	  break;
	case 5:
	  l.vertices.second.i = i * -1;
	  l.vertices.second.j = i * 2;
	  break;
	case 6:
	  l.vertices.second.i = i * -1;
	  l.vertices.second.j = i * 1;
	  break;
	case 7:
	  l.vertices.second.i = i * -2;
	  l.vertices.second.j = i * 1;
	  break;
      }
      
      flatLinks.push_back(l);
    }
  }
  
  //Create and push links for ledge terrain
  vector<vector<Link>> ledgeLinks;
  vector<float> paramAngles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435,
      180, 180 + 26.5651, 180 + 45, 180 + 63.4349, 180 + 90, 180 + 116.565, 180 + 135, 180 + 153.435};
  
  //Only need to create and push snake climbing links
  Link snakeLedgeLink(make_pair(NodeSpec(1, 0, 0, 0), NodeSpec(1, 0, 0, 0)),
		     make_pair(vector<bool>({1, 0, 1, 1}), vector<bool>({1, 0, 1, 1})), 0, 3.0);
  for (int theta = 0; theta < 16; theta++)
  {
    Link l = snakeLedgeLink;
    
    if (theta >= 8)
    {
      l.vertices.first.th = theta % 8;
      l.vertices.second.th = theta % 8;
    }
    else
    {
      l.vertices.first.th = theta + 8;
      l.vertices.second.th = theta + 8;
    }
    
    //l.vertices.first.th = theta % 8;
    //l.vertices.second.th = theta % 8;
    
    //Only do climbing direction now
    int i = 1;
    
    l.vertices.first.j = i * round(22 * cos(paramAngles[theta] * M_PI / 180));
    l.vertices.first.i = i * round(22 * sin(paramAngles[theta] * M_PI / 180));
    l.vertices.second.j = -l.vertices.first.j;
    l.vertices.second.i = -l.vertices.first.i;
    if (theta < 8)
    {
      l.action = 1;
    }
    else
    {
      l.action = 1;
    }
    
    //cout << "Ledge Indices: " << l.vertices.first.j << ", " << l.vertices.first.i << endl;
    
    ledgeLinks.push_back({l});
  }
  
  //Dolphin links for wavy feature
  
  //Create and push snake links
  Link dolphinLink(make_pair(NodeSpec(2, 0, 0, 0), NodeSpec(2, 0, 0, 0)),
		     make_pair(vector<bool>({1, 0, 1, 0}), vector<bool>({1, 0, 1, 0})), 0, 1);
  for (int theta = 0; theta < 16; theta++)
  {
    //for (int i = -1; i <= 1; i += 2)
    //{
      Link l = dolphinLink;
      l.vertices.first.th = theta;// + 8 * (i < 0);
      l.vertices.second.th = theta;// + 8 * (i < 0);
      
      int i = 1;
      
      if (theta < 8) i = -1;
      
      //Forward-Backward Neighbors
      switch (theta % 8)
      {
	case 0:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 0;
	  break;
	case 1:
	  l.vertices.second.i = i * 2;
	  l.vertices.second.j = i * 1;
	  break;
	case 2:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 1;
	  break;
	case 3:
	  l.vertices.second.i = i * 1;
	  l.vertices.second.j = i * 2;
	  break;
	case 4:
	  l.vertices.second.i = i * 0;
	  l.vertices.second.j = i * 1;
	  break;
	case 5:
	  l.vertices.second.i = i * -1;
	  l.vertices.second.j = i * 2;
	  break;
	case 6:
	  l.vertices.second.i = i * -1;
	  l.vertices.second.j = i * 1;
	  break;
	case 7:
	  l.vertices.second.i = i * -2;
	  l.vertices.second.j = i * 1;
	  break;
      }
      
      //Push flat dolphin action first
      l.action = 0;
      l.obs.first[3] = 1;
      l.obs.second[3] = 1;
      l.cost = 1;
      flatLinks.push_back(l);
      
      //Push wavy dolpin action
      l.action = 1;
      l.obs.first[3] = 0;
      l.obs.second[3] = 0;
      l.cost = 2;
      flatLinks.push_back(l);
      wavyLinks.push_back(l);
    //}
  }
  
  featLinks.push_back({flatLinks});
  featLinks.push_back(ledgeLinks);
  featLinks.push_back({wavyLinks});
  
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
  
  chrono::steady_clock::time_point start = chrono::steady_clock::now();
  
  Flattenize(grid, &features, mapCush, 3, 0.00003);
  
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  cout << "Flattenize Time: " << chrono::duration_cast<chrono::microseconds>(t1 - start).count() / 1000000.0 << endl;
  
  vector<std_msgs::Float32MultiArray> partitions = Snippetize(grid, mapCush);
  
  cout << "Partitions: " << partitions.size() << endl;
  
  env_characterization::classify_map srv;
  srv.request.partitions = partitions;
  
  vector<env_characterization::Feature> featureMap;
  
  chrono::steady_clock::time_point t2;
  
  if (client.call(srv))
  {
    cout << "Service Responded" << endl;
    
    t2 = chrono::steady_clock::now();

    cout << "Classification Time: " << chrono::duration_cast<chrono::microseconds>(t2 - t1).count() / 1000000.0 << endl;
    
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
  
  //Translate featureMap to bloatFeats
  vector<int> bloatFeats;
  
  int graphHeight = grid.getSize()[0] - 20;
  int graphWidth = grid.getSize()[1] - 20;
  
  configMap planGraph = bloatMap(featureMap, graphHeight, graphWidth, configList, angles, 3, reconRad);
  
  for (int i = 0; i < planGraph[0][0].size(); i++)
  {
    for (int j = 0; j < planGraph[0][0][i].size(); j++)
    {
      float obs = 0;
      for (int c = 0; c < planGraph.size(); c++)
      {
	for (int th = 0; th < planGraph[c].size(); th++)
	{
	  if (planGraph[2][6][i][j][0])
	  {
	    obs = 1;
	    break;
	  }
	  else if(planGraph[2][6][i][j][3])
	  {
	    obs = 2;
	    break;
	  }
	}
	if (obs) break;
      }
      if (obs > 0)
      {
	grid.at("bloat", Index(i, j) + Index(10, 10)) = obs;
      }
    }
  }
  
  chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
  
  cout << "Bloat Time: " << chrono::duration_cast<chrono::microseconds>(t3 - t2).count() / 1000000.0 << endl;
  
  //Planning Graph Construction
  
  cout << "Graph Sizes: car angles " << planGraph[0].size() << "; snake angles " << planGraph[1].size() << endl;
  
  cout << "Starting Node Graph Generation" << endl;
  
  vector<Node> nodeGraph;
  configNodeMap nodeGrid;
  
  CreateGraph(nodeGrid, nodeGraph, planGraph, featureMap, graphHeight, graphWidth, configList, angles.size(), featLinks);
  
  nodeGridPtr = &nodeGrid;
  nodeGraphPtr = &nodeGraph;
  
  cout << "Finished Node Graph Creation" << endl;
  cout << "Graph Size: " << nodeGraph.size() << endl;
  
  chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
  
  cout << "Graph Creation Time: " << chrono::duration_cast<chrono::microseconds>(t4 - t3).count() / 1000000.0 << endl;
  
  cout << "Begin Dijkstra Path Planning" << endl;
  
  //Dijkstra Path Planning
  Index idxStart;
  grid.getIndex(Position(0.5, -0.5), idxStart);
  
  idxStart -= Index(10, 10);
  
  cout << "Start Node: " << nodeGrid[0][0][idxStart[0]][idxStart[1]] << endl;
  Node* startNode = &nodeGraph[nodeGrid[0][0][idxStart[0]][idxStart[1]]];
  planPath(nodeGraph, startNode);
  
  cout << "Finished Path Planning" << endl;
  
  chrono::steady_clock::time_point totStop = chrono::steady_clock::now();
  cout << "Path Planning Time: " << chrono::duration_cast<chrono::microseconds>(totStop - t4).count() / 1000000.0 << endl;
  cout << "Total Processing Time: " << chrono::duration_cast<chrono::microseconds>(totStop - start).count() / 1000000.0 << endl;
  
  float res = grid.getResolution();
  
  cout << "Resolution: " << res << endl;
  
  //Publish path msg
  
  Position zeroPos;
  grid.getPosition(Index(10, 10), zeroPos);
  
  Node* goalNode = &nodeGraph[nodeGrid[1][6][145][112]];
  Node* curParent = goalNode->parent;
  
  env_characterization::PathNodeArray path;
  env_characterization::PathNode pathNode;
  pathNode.config = goalNode->config;
  pathNode.theta = angles[goalNode->pose.theta] * 3.141592 / 180;
  pathNode.action = goalNode->action;
  
  geometry_msgs::Pose waypt;
  waypt.position.x = zeroPos[0] - goalNode->pose.x * res + res / 2;
  waypt.position.y = zeroPos[1] - goalNode->pose.y * res + res / 2;
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
    
    waypt.position.x = zeroPos[0] - curParent->pose.x * res + res / 2;
    waypt.position.y = zeroPos[1] - curParent->pose.y * res + res / 2;
    waypt.position.z = 0;
    pathNode.pose = waypt;
    
    path.path.push_back(pathNode);
    
    curParent = curParent->parent;
  }
  
  reverse(path.path.begin(), path.path.end());
  
  pathPtr = &path;
  
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
  pathLine.header.frame_id = "map";
  pathLine.id = linez.size() + 2;
  pathLine.color.r = 1.0;
  pathLine.color.g = 1.0;
  pathLine.color.b = 1.0;
  pathLine.scale.x = 0.006;
  
  cout << occ_grid.info.origin.position.y << endl;
  curParent = goalNode->parent;
  
  geometry_msgs::Point p;
  p.x = zeroPos[0] - goalNode->pose.x * res + res / 2;
  p.y = zeroPos[1] - goalNode->pose.y * res + res / 2;
  p.z = (goalNode->config) * 0.2 + 0.01;
  
  cout << goalNode->pose.x << ", " << goalNode->pose.y << " : " << goalNode->config << endl;
  cout << "Cost: " << goalNode->cost << endl;
  while (curParent != NULL)
  {
    //cout << curParent->pose.x << ", " << curParent->pose.y << " : " << curParent->config << endl;
    
    pathLine.points.push_back(p);
    p.x = zeroPos[0] - curParent->pose.x * res + res / 2;
    p.y = zeroPos[1] - curParent->pose.y * res + res / 2;
    p.z = (curParent->config) * 0.2 + 0.01;
    pathLine.points.push_back(p);
    
    curParent = curParent->parent;
  }
  
  pathVizPtr = &pathLine;
  
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
    
    path_viz_pub.publish(*pathVizPtr);
    path_pub.publish(*pathPtr);
    
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}