#include <cstdio>
#include <stdlib.h>
#include <cmath>
#include <boost/config/no_tr1/complex.hpp>
#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>

#include <cmvision/Blobs.h>
#include <cmvision/Blob.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>

using namespace std;

sensor_msgs::PointCloud2 depth;
pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
int cloud_counter = 0;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cloud_counter < 10)
  {
    cloud_counter++;
    depth = *msg;

    //std::cout<<msg->height<<" ";
    pcl::PCLPointCloud2 pcl_pc2;
    //pcl_conversions::toPCL((sensor_msgs::PointCloud2&)msg, pcl_pc2);
    pcl_conversions::toPCL(*msg, pcl_pc2);
      //Actually convert the PointCloud2 message into a type we can reason about
    pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_calibration");
  
  ros::NodeHandle node;
  ros::Subscriber cloud_sub = node.subscribe("/minnow/camera/depth_registered/points", 1, cloud_cb);
  
  tf::TransformListener listenerTF;
  tf::TransformBroadcaster br;
  
  geometry_msgs::Vector3 v;
  pcl::PointXYZ point;
  
  while (true)
  {
    while (ros::ok() && cloud_counter < 10)
    {
      ros::spinOnce();
    }
    
    point = pcl_cloud.at(319, 240);
    v.y = point.y;
    v.x = point.x;
    v.z = point.z;
    
    if (!(v.x == 0 && v.y == 0 && v.z == 0) && point.x == point.x)
    {
      break;
    }
    
    ROS_ERROR("Failed cloud lookup");
    cloud_counter--;
    ros::spinOnce();
  }
  
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud (pcl_cloud.makeShared());

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  searchPoint.x = v.x;
  searchPoint.y = v.y;
  searchPoint.z = v.z;
  
  int K = 500;
  
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
	    << " " << searchPoint.y 
	    << " " << searchPoint.z
	    << ") with K=" << K << std::endl;
	    
  
  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    /*for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
		<< " " << cloud->points[ pointIdxNKNSearch[i] ].y 
		<< " " << cloud->points[ pointIdxNKNSearch[i] ].z 
		<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;*/
  }

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  
  Eigen::Vector4f plane_parameters;
  float nx, ny, nz, curvature;
  
  ne.computePointNormal(*cloud, pointIdxNKNSearch, nx, ny, nz, curvature);
  
  cout << "Dat Vector: " << nx << ", " << ny << ", " << nz << endl;    
  
  tf::Quaternion quat;
  
  //This function actually performs Yaw, Pitch, Roll on Body Coordinates
  quat.setRPY(0, -atan2(nz, sqrt(pow(ny, 2) + pow(nx, 2))), atan2(ny, nx));
  
  geometry_msgs::Quaternion rotation;
  tf::quaternionTFToMsg(quat, rotation);
  
  geometry_msgs::Pose pose;
  pose.position.x = v.x;
  pose.position.y = v.y;
  pose.position.z = v.z;
  pose.orientation = rotation;
  
  tf::Transform colorTF;
  tf::Transform rotTF;
  
  colorTF.setOrigin( tf::Vector3(v.x, v.y, v.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  if (q.w() == q.w())
  {
    colorTF.setRotation(q);
  }
  else
  {
    q.setW(1);
    cout << "NANNNNN" << endl;
  }
  
  q.setRPY(0, -DEG2RAD(90), DEG2RAD(180));
  rotTF.setRotation(q);
  
  cloud_sub.shutdown();
  
  tf::Transform cal_tf;
  tf::StampedTransform calibration;
  
  tf::TransformListener listener;
  
  while (ros::ok())
  {
    br.sendTransform(tf::StampedTransform(colorTF, ros::Time::now(), "camera_rgb_optical_frame", "floor_norm"));
    br.sendTransform(tf::StampedTransform(rotTF, ros::Time::now(), "floor_norm", "cal_frame"));
    
    try {
      ros::Time gtime = ros::Time(0);
      listener.waitForTransform("cal_frame", "camera_link", gtime, ros::Duration(2));
      listener.lookupTransform("cal_frame", "camera_link", gtime, calibration);
      ROS_INFO("Transform Found");
      
      cal_tf.setOrigin(tf::Vector3(-0.075, calibration.getOrigin().getY(), calibration.getOrigin().getZ()));
      cal_tf.setRotation(calibration.getRotation());
      break;
    } catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    }
    
    ros::spinOnce();
  }
  
  ros::Rate rate(50);
  
  tf::Matrix3x3 m(calibration.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  
  ROS_INFO_STREAM("Calibrated Transform: -0.075 " << calibration.getOrigin().getY() << " " << calibration.getOrigin().getZ() << " " << roll
    << " " << pitch << " " << yaw);
  
  while (ros::ok())
  {
    //br.sendTransform(tf::StampedTransform(colorTF, ros::Time::now(), "camera_rgb_optical_frame", "floor_norm"));
    //br.sendTransform(tf::StampedTransform(rotTF, ros::Time::now(), "floor_norm", "cal_frame"));
    br.sendTransform(tf::StampedTransform(cal_tf, ros::Time::now() + ros::Duration(0.4), "base_link", "camera_link"));
    rate.sleep();
  }
  
  return 0;
}