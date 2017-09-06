#include <stdio.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

bool flow = false;
bool astra = true;
ros::Publisher cloud_pub;
ros::Publisher image_pub;
ros::Publisher info_pub;
std::string april_cam = "camera_link";
sensor_msgs::CameraInfo info_msg;

void switch_cb(const std_msgs::BoolConstPtr& msg)
{
	flow = msg->data;
	std::cout << flow << std::endl;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	if (flow)
	{
		sensor_msgs::PointCloud2 cloud(*msg);
		cloud_pub.publish(cloud);
	}	
}

void astraInfo_cb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (astra)
  {
    //info_pub.publish(*msg);
    info_msg = *msg;
  }
}

void usbInfo_cb(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (!astra)
  {
    //info_pub.publish(*msg);
    info_msg = *msg;
  }
}

void imageSwitch_cb(const std_msgs::BoolConstPtr& msg)
{
  astra = msg->data;
  std::cout << "Image Source Astra: " << astra << std::endl;
  if (astra)
  {
    april_cam = "camera_link";
  }
  else
  {
    april_cam = "usb_cam";
  }
}

void astra_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if (astra)
  {
    sensor_msgs::Image newMsg = *msg;
    info_msg.header.stamp = newMsg.header.stamp;
    image_pub.publish(newMsg);
    info_pub.publish(info_msg);
  }
}

void usb_cb(const sensor_msgs::ImageConstPtr& msg)
{
  if (!astra)
  {
    sensor_msgs::Image newMsg = *msg;
    info_msg.header.stamp = newMsg.header.stamp;
    image_pub.publish(newMsg);
    info_pub.publish(info_msg);
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cloud_throttle");
	ros::NodeHandle node;
	ros::Rate rate(70.0);
	
	tf::TransformListener listener;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform transform2;
	tf::Transform cam_transform;
	tf::StampedTransform stransform;

	cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/cloud_in", 15);

	ros::Subscriber switch_sub = node.subscribe<std_msgs::Bool>("/cloud_throttle", 15, switch_cb);
	ros::Subscriber cloud_sub = node.subscribe<sensor_msgs::PointCloud2>("/minnow/camera/depth/points", 15, cloud_cb);
	ros::Subscriber astra_sub = node.subscribe<sensor_msgs::Image>("/minnow/camera/rgb/image_rect_color", 15, astra_cb);
	ros::Subscriber cam_sub = node.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 15, usb_cb);
	ros::Subscriber imageSwitch_sub = node.subscribe<std_msgs::Bool>("/april_source", 15, imageSwitch_cb);
	
	ros::Subscriber astraInfo_sub = node.subscribe<sensor_msgs::CameraInfo>("/minnow/camera/rgb/camera_info", 15, astraInfo_cb);
	ros::Subscriber usbInfo_sub = node.subscribe<sensor_msgs::CameraInfo>("/usb_cam/camera_info", 15, usbInfo_cb);
	
	image_pub = node.advertise<sensor_msgs::Image>("april_cam/image_raw", 15);
	info_pub = node.advertise<sensor_msgs::CameraInfo>("april_cam/camera_info", 15);
	
	while (ros::ok())
	{
	  /*try
	  {
	    listener.waitForTransform("map", "odom", ros::Time::now(), ros::Duration(0.1));
	    listener.lookupTransform("map", "odom", ros::Time::now(), stransform);
	    
	    transform.setOrigin(stransform.getOrigin());
	    transform.setRotation(stransform.getRotation());
	  }
	  catch (tf::TransformException &ex)
	  {
	    
	  }
	  
	  /*try
	  {
	    listener.lookupTransform("odom", "base_footprint", ros::Time::now(), stransform);
	    
	    transform2.setOrigin(stransform.getOrigin());
	    transform2.setRotation(stransform.getRotation());
	  }
	  catch (tf::TransformException &ex)
	  {
	    
	  }*/
	  
	  cam_transform.setOrigin(tf::Vector3(0, 0, 0));
	  cam_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
	  br.sendTransform(tf::StampedTransform(cam_transform, ros::Time::now(), april_cam, "april_cam"));
	  //br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "odom", "base_footprint"));
	  rate.sleep();
	  ros::spinOnce();
	}

	ros::spin();
}
