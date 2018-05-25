#include <ros/ros.h>
#include <bits/stdc++.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_octomap/GridMapOctomapConverter.hpp>
#include <grid_map_core/SubmapGeometry.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_ros/conversions.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <nav_msgs/OccupancyGrid.h>

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace grid_map;
using namespace std;

grid_map_msgs::GridMap debug_msg;
grid_map_msgs::GridMap grid_msg;
grid_map_msgs::GridMap snip_msg;
GridMap* gridPtr;

string data_dir = "/home/jonathan/catkin_ws/train_db/";
int nLabels = 2;
vector<float> angles = {0, 26.5651, 45, 63.4349, 90, 116.565, 135, 153.435,
  180, 180 + 26.5651, 180 + 45, 180 + 63.4349, 180 + 90, 180 + 116.565, 180 + 135, 180 + 153.435};
  
vector<vector<int>> sample_counts(2, vector<int>(angles.size(), 0));

ros::Publisher snip_pub;

float map_size = 0.12;
int snip_size = 12;

vector<int> sampleCount(nLabels, 0);

//string feature("flat");

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

void click_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  geometry_msgs::Pose pose = msg->pose.pose;
  //double yaw = pose.orientation.z / pose.orientation.w;
  double yaw = tf::getYaw(pose.orientation);
  
  cout << "Height at (" << pose.position.x << ", " << pose.position.y << ") (" << yaw << ") : ";
  Position pos(pose.position.x, pose.position.y);
  if (gridPtr->isInside(pos))
  {
    cout << gridPtr->atPosition("elevation", pos) << endl;
    
    bool success;
    GridMap snip = gridPtr->getSubmap(pos, Length((snip_size - 1) * gridPtr->getResolution(),
						  (snip_size - 1) * gridPtr->getResolution()), success);
    //GridMapRosConverter::toMessage(snip, snip_msg);
    //snip_pub.publish(snip_msg);
    
    ros::Rate r(50);
    for (float th = 0; th < 359.9; th+= 10)
    {
      GridMap snip_rot = snip;
      double angle = (360 - th) * M_PI / 180 + yaw;
      
      vector<double> errors;
      for (int i = 0; i < angles.size(); i++)
      {
	errors.push_back(min(abs(th - angles[i]), 360 - abs(th - angles[i])));
      }
      int thID = distance(errors.begin(), min_element(errors.begin(), errors.end()));
      
      Eigen::Rotation2Dd t(angle);
      for (GridMapIterator it(snip_rot); !it.isPastEnd(); ++it)
      {
	Position position;
	snip_rot.getPosition(*it, position);
	
	Eigen::Vector2d vec = position - snip_rot.getPosition();
	Eigen::Vector2d pos_rot = t * vec;
	
	position = snip_rot.getPosition() + pos_rot;
	
	if (gridPtr->isInside(position))
	{
	  snip_rot.at("elevation", *it) = gridPtr->atPosition("elevation", position);
	}
      }
      
      GridMapRosConverter::toMessage(snip_rot, snip_msg);
      snip_pub.publish(snip_msg);
      
      r.sleep();
    }
      
    int feat;
    cout << "Enter Feature # (-1 to discard): ";
    cin >> feat;
    
    //Now go through again to write images
    if (feat > -1)
    {
      for (float heightOff = 0.0; heightOff <= 0.1; heightOff += 0.02)
      {
	for (float th = 0; th < 359.9; th+= 5)
	{
	  GridMap snip_rot = snip;
	  double angle = (360 - th) * M_PI / 180 + yaw;
	  
	  vector<double> errors;
	  for (int i = 0; i < angles.size(); i++)
	  {
	    errors.push_back(min(abs(th - angles[i]), 360 - abs(th - angles[i])));
	  }
	  int thID = distance(errors.begin(), min_element(errors.begin(), errors.end()));
	  
	  Eigen::Rotation2Dd t(angle);
	  for (GridMapIterator it(snip_rot); !it.isPastEnd(); ++it)
	  {
	    Position position;
	    snip_rot.getPosition(*it, position);
	    
	    Eigen::Vector2d vec = position - snip_rot.getPosition();
	    Eigen::Vector2d pos_rot = t * vec;
	    
	    position = snip_rot.getPosition() + pos_rot;
	    
	    if (gridPtr->isInside(position))
	    {
	      snip_rot.at("elevation", *it) = gridPtr->atPosition("elevation", position) + heightOff;
	    }
	  }
	  
	  cv::Mat originalImage;
	  GridMapCvConverter::toImage<uint8_t, 1>(snip_rot, "elevation", CV_8UC1, 0.0, 1.0, originalImage);
	  string img_file;
	  if (feat == 0)
	  {
	    img_file = data_dir + to_string(feat) + "_" + to_string(sample_counts[feat][0]++) + ".png";
	  }
	  else
	  {
	    img_file = data_dir + to_string(feat) + "_" + to_string(thID) + "_" + to_string(sample_counts[feat][thID]++) + ".png";
	  }
	  cv::imwrite(img_file, originalImage);
	}
      }
      
      cout << "Samples written to db" << endl;
    }
    else cout << "Discarded" << endl;
  }
  else
  {
    cout << "isn't valid" << endl;
  }
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "hand_labelling");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  ros::Publisher occ_pub = nh.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1, true);
  ros::Publisher debug_pub = nh.advertise<grid_map_msgs::GridMap>("/debug_grid", 15, true);
  
  ros::Publisher grid_pub = nh.advertise<grid_map_msgs::GridMap>("/elevation", 1);
  snip_pub = nh.advertise<grid_map_msgs::GridMap>("/snip", 1);
  
  ros::Subscriber click_sub = nh.subscribe("/initialpose", 1, click_cb);
  
  ros::Rate rate(5);
  
  cout << "Scanning Database" << endl;
  
  for (int i = 0; i < sample_counts.size(); i++)
  {
    for (int j = 0; j < sample_counts[i].size(); j++)
    {
      int n = 0;
      
      while (true)
      {
	string filename = data_dir;
	
	if (i == 0)
	{
	  filename += to_string(i) + "_" + to_string(n) + ".png";
	}
	else
	{
	  filename += to_string(i) + "_" + to_string(j) + "_" + to_string(n) + ".png";
	}
	
	ifstream f(filename);
	if (f.good())
	{
	  f.close();
	  n++;
	}
	else break;
      }
      
      sample_counts[i][j] = n;
      cout << n << ", ";
      
      if (i == 0) break;
    }
    cout << endl;
  }
  
  string filename;
  cout << "Enter Filename in /home/jonathan/catkin_ws:" << endl;
  cin >> filename;
  string path = "/home/jonathan/catkin_ws/" + filename;
  octomap::OcTree tree(filename);//"/home/jonathan/catkin_ws/env_2_2.bt");
  
  octomap::OcTree tree2(0.02);
  
  for (octomap::OcTree::leaf_iterator iter = tree.begin_leafs(14); iter != tree.end_leafs(); iter++)
  {
    //octomap::OcTreeNode node = *iter;
    if (tree.isNodeOccupied(*iter))
    {
      octomap::point3d point = iter.getCoordinate() - octomap::point3d(0.0, 0.0, 0.06);
      point.z() = max(point.z(), 0.0f);
      tree2.setNodeValue(point, iter->getLogOdds());
    }
  }
  
  GridMap grid({"elevation"});
  grid.setFrameId("map");
  GridMapOctomapConverter::fromOctomap(tree2, "elevation", grid);
  
  GridMap grid_square = grid;
  grid_square.setGeometry(Length(2.57, 2.57), grid.getResolution(), grid.getPosition());
  
  for (GridMapIterator it(grid_square); !it.isPastEnd(); ++it)
  {
    grid_square.at("elevation", *it) = 0.005;
  }
  
  grid_square.addDataFrom(grid, false, true, true);
  grid = grid_square;
  
  float maxVal = 0;
  for (GridMapIterator iter(grid); !iter.isPastEnd(); ++iter)
  {
    maxVal = max(maxVal, grid.at("elevation", *iter));
  }
  cout << "Maximum Height: " << maxVal << endl;  
  
  cv::Mat originalImage;
  //path.replace(path.end()-3, path.end(), ".png");
  path = "/home/jonathan/catkin_ws/env_map.png";
  GridMapCvConverter::toImage<uint8_t, 1>(grid, "elevation", CV_8UC1, 0.0, 1.0, originalImage);
  cv::imwrite(path, originalImage);
  
  originalImage = cv::imread(path);
  GridMapCvConverter::addLayerFromImage<uint8_t, 1>(originalImage, "elevation", grid);
  grid.setPosition(Position(0.0, 0.0));
  
  gridPtr = &grid;
  
  GridMapRosConverter::toMessage(grid, grid_msg);
  
  while (ros::ok())
  {
    grid_pub.publish(grid_msg);
    snip_pub.publish(snip_msg);
    
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}