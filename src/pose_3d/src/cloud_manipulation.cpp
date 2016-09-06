#include <stdio.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher cloud_pub;

void cloud_cb(sensor_msgs::PointCloud2ConstPtr msg)
{
  if (!msg->is_dense) std::cout << "oooookkkk" << std::endl;
  
  sensor_msgs::PointCloud2 cloud = *msg;
  
  cloud.is_dense = true;
  
//   for (int i = 0; i < cloud.height * cloud.width; i++)
//   {
//     if (cloud.data[i] <= 0)
//     {
//       cloud.data[i] = 1000;
//     }
//   }
  
  cloud_pub.publish<sensor_msgs::PointCloud2>(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_3d");
  
  ros::NodeHandle node;
  
  ros::Rate rate(10.0);
  
  cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/mod_cloud", 15);
  
  ros::Subscriber sub = node.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 15, cloud_cb);
  
  ros::spin();
}