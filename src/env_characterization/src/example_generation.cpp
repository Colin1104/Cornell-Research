#include <ros/ros.h>
#include <bits/stdc++.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <nav_msgs/OccupancyGrid.h>

#include <cmath>

using namespace grid_map;
using namespace std;

grid_map_msgs::GridMap debug_msg;

float map_size = 0.12;

int nLabels = 2;
vector<int> sampleCount(nLabels, 0);

//string feature("flat");

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

nav_msgs::OccupancyGrid writeImg(string feature, double theta, float offsetY, float offsetX)
{
  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(map_size, map_size), 0.01);
  
  // Work with grid map in a loop.
  ros::Rate rate(30.0);
  
  double illumination = 127;
  
  // Add data to grid map.
  ros::Time time = ros::Time::now();
  
  int label = 0;
  
  double res = map.getResolution();
  for (GridMapIterator it(map); !it.isPastEnd(); ++it)
  {
    Position position;
    map.getPosition(*it, position);
    
    double intensity = 0;
    
    //Noisy flat ground
    if (feature == "flat")
    {
      intensity = double(rand()) / double(RAND_MAX) / 100;
    }
    
    //Ledge Feature
    if (feature == "ledge")
    {
      if (abs(offsetY) < 0.02 && abs(offsetX) < 0.02)
      {
	label = 1;
      }
      
      double y = position.x() * -sin(theta) + position.y() * cos(theta);
      double x = position.x() * cos(theta) + position.y() * sin(theta);
      if (y >= offsetY && abs(x - offsetX) <= map_size / (2 * max(abs(cos(theta)), abs(sin(theta)))))
      {
	intensity = 0.1;
      }
      else
      {
	intensity = 0.0;
      }
    }
    
    map.at("elevation", *it) = intensity;
  }
  
  cv::Mat originalImage;
  GridMapCvConverter::toImage<uint8_t, 1>(map, "elevation", CV_8UC1, -0.5, 0.5, originalImage);
  //string path = "/home/jonathan/image_db/" + feature + to_string(nImg++) + ".png";
  string path = "/home/jonathan/env_characterization_db/" + to_string(label) + "_" + to_string(sampleCount[label]++) + ".png";
  cv::imwrite(path, originalImage);
  
  // Publish grid map.
  /*map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  publisher.publish(message);
ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
      
  ros::Duration(0.01).sleep();*/
  
  nav_msgs::OccupancyGrid occ_map;
  GridMapRosConverter::toOccupancyGrid(map, "elevation", -0.5, 0.5, occ_map);
  
  GridMapRosConverter::toMessage(map, debug_msg);
  
  return occ_map;
}

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "example_generation");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
  ros::Publisher occ_pub = nh.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1, true);
  ros::Publisher debug_pub = nh.advertise<grid_map_msgs::GridMap>("/debug_grid", 15, true);
  
  ros::Rate rate(50);
  
  nav_msgs::OccupancyGrid occ_map;
  
  for (int i = 0; i < 45; i++)
  {
    occ_map = writeImg("flat", 0, 0.0, 0.0);
  }
  for (int i = 0; i < 360; i++)
  {
    double theta = deg2rad(double(i));
    occ_map = writeImg("ledge", theta, 0.0, 0.0);
    
    for (float offsetY = -0.08; offsetY < 0.081; offsetY += 0.04)
    {
      if (abs(offsetY) < 0.01) continue;
      
      occ_map = writeImg("ledge", theta, offsetY, 0.0);
    }
    for (float offsetX = -0.08; offsetX < 0.081; offsetX += 0.04)
    {
      if (abs(offsetX) < 0.01) continue;
      
      occ_map = writeImg("ledge", theta, 0.0, offsetX);
    }
  }
  
  cout << "Wellp" << endl;
  
  while (nh.ok())
  {
    // Publish occ map.
    occ_pub.publish(occ_map);
    
    //Publish debug grid_map
    debug_pub.publish(debug_msg);

    // Wait for next cycle.
    rate.sleep();
  }
  //}

  return 0;
}