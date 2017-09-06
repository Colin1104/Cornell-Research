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

class colorTracker
{
  private:
    geometry_msgs::Vector3 trackedObject;
    geometry_msgs::Pose trackedPose;
    sensor_msgs::PointCloud2 depth;
    pcl::PointCloud < pcl::PointXYZ > pcl_cloud;
    
    //pcl::PointCloud< pcl::PointXYZ >::Ptr cloud = new pcl::PointCloud<pcl::PointXYZ>;
    
    tf::Transform colorTF;   

    tf::TransformListener listenerTF;
    tf::TransformBroadcaster br;
    
    ros::NodeHandle *n;
    
  public:
    ros::Publisher blobPub;
    ros::Publisher blob2Pub;
    ros::Publisher blob3Pub;
    ros::Publisher point_pub;
    ros::Publisher pose_pub;
    
    bool color_switch;
    
    void blobCallback(const cmvision::Blobs::ConstPtr& msg)
    {
      if (!color_switch) return;
      cmvision::Blob colorObj;
      cmvision::Blob blob;

      if(msg->blob_count==0)
      {
	ROS_INFO("No blobs detected.");
	return;
      }

      blob = msg->blobs[0];

      colorObj = blob;
      
      printf("x: %u, y: %u, area: %u\n", blob.x, blob.y, blob.area);
      
      geometry_msgs::Vector3 blobby;
      blobby.x = int(blob.x) - 320;
      blobby.y = 480 - blob.y;
      blobby.z = blob.area;
      
      blobPub.publish<geometry_msgs::Vector3>(blobby);
      
      //trackedPose = getPose(blob.x, blob.y);
      trackedPose = getPose(320, 240);
      trackedObject = geometry_msgs::Vector3();
      trackedObject.x = trackedPose.position.x;
      trackedObject.y = trackedPose.position.y;
      trackedObject.z = trackedPose.position.z;
      if (trackedObject.z == trackedObject.z)
      {
           cout << trackedObject.x << " : " << trackedObject.y << " : " << trackedObject.z << endl;
           /*printf("Position of the tracked object >>> x: %f; y: %f; z: %f\n", 
	     	trackedObject.linear.x, trackedObject.linear.y, trackedObject.linear.z);*/
           tfPublisher("/pinkObj");
      }
    }
    
    void blob2Callback(const cmvision::Blobs::ConstPtr& msg)
    {
      if (!color_switch) return;
      cmvision::Blob colorObj;
      cmvision::Blob blob;

      if(msg->blob_count==0)
      {
	ROS_INFO("No blobs detected.");
	return;
      }

      blob = msg->blobs[0];

      colorObj = blob;

      printf("x: %u, y: %u, area: %u\n", blob.x, blob.y, blob.area);
      
      geometry_msgs::Vector3 blobby;
      blobby.x = int(blob.x) - 320;
      blobby.y = 480 - blob.y;
      blobby.z = blob.area;
      
      blob2Pub.publish<geometry_msgs::Vector3>(blobby);
      
      trackedPose = getPose(blob.x, blob.y);
      trackedObject = geometry_msgs::Vector3();
      trackedObject.x = trackedPose.position.x;
      trackedObject.y = trackedPose.position.y;
      trackedObject.z = trackedPose.position.z;
      cout << "Drrrp" << endl;
      if (trackedObject.z == trackedObject.z)
      {
           cout << "Blue: " << trackedObject.x << " : " << trackedObject.y << " : " << trackedObject.z << endl;
           /*printf("Position of the tracked object >>> x: %f; y: %f; z: %f\n", 
	     	trackedObject.linear.x, trackedObject.linear.y, trackedObject.linear.z);*/
           tfPublisher("/blueObj");
      }
    }
    
    void blob3Callback(const cmvision::Blobs::ConstPtr& msg)
    {
      if (!color_switch) return;
      cmvision::Blob colorObj;
      cmvision::Blob blob;

      if(msg->blob_count==0)
      {
	ROS_INFO("No blobs detected.");
	return;
      }

      blob = msg->blobs[0];

      colorObj = blob;

      printf("x: %u, y: %u, area: %u\n", blob.x, blob.y, blob.area);
      
      geometry_msgs::Vector3 blobby;
      blobby.x = int(blob.x) - 320;
      blobby.y = 480 - blob.y;
      blobby.z = blob.area;
      
      blob3Pub.publish<geometry_msgs::Vector3>(blobby);
      
      trackedPose = getPose(blob.x, blob.y);
      trackedObject = geometry_msgs::Vector3();
      trackedObject.x = trackedPose.position.x;
      trackedObject.y = trackedPose.position.y;
      trackedObject.z = trackedPose.position.z;
      if (trackedObject.z == trackedObject.z)
      {
           cout << "Blue: " << trackedObject.x << " : " << trackedObject.y << " : " << trackedObject.z << endl;
           /*printf("Position of the tracked object >>> x: %f; y: %f; z: %f\n", 
	     	trackedObject.linear.x, trackedObject.linear.y, trackedObject.linear.z);*/
           tfPublisher("/greenObj");
      }
    }
    
    geometry_msgs::Pose getPose(int x, int y)
    {
      geometry_msgs::Vector3 v;
      pcl::PointXYZ point = pcl_cloud.at(x, y);
      v.x = point.x;
      v.y = point.y;
      v.z = point.z;
      
      //cout << point.x << " ... " << point.y << " ... " << point.z << endl;
      
      if ((v.x == 0 && v.y == 0 && v.z == 0) || point.x != point.x)
      {
	geometry_msgs::Pose pose;

	return pose;
      }

      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud (pcl_cloud.makeShared());

      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

      kdtree.setInputCloud (cloud);

      pcl::PointXYZ searchPoint;

      searchPoint.x = v.x;
      searchPoint.y = v.y;
      searchPoint.z = v.z;
      
      int K = 100;

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
      
      /*geometry_msgs::Vector3Stamped vec;
      geometry_msgs::Vector3Stamped vec_out;
      vec.header.frame_id = "pinkObj";
      vec.vector.x = nx;
      vec.vector.y = ny;
      vec.vector.z = nz;
      
      tf::TransformListener listener;
      ros::Time gTime = ros::Time(0);
      listener.waitForTransform("camera_rgb_optical_frame", "pinkObj", gTime, ros::Duration(2.0));
      try{
      listener.transformVector("pinkObj", vec, vec_out);
      }
      catch(tf::TransformException& ex){
	  ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
	}
      
      cout << vec_out.vector.x << ", " << vec_out.vector.y << ", " << vec_out.vector.z << endl;*/
      
      
      //This function actually performs Yaw, Pitch, Roll on Body Coordinates
      quat.setRPY(0, -atan2(nz, sqrt(pow(ny, 2) + pow(nx, 2))), atan2(ny, nx));
      
      geometry_msgs::Quaternion rotation;
      tf::quaternionTFToMsg(quat, rotation);
      
      geometry_msgs::Pose pose;
      pose.position.x = v.x;
      pose.position.y = v.y;
      pose.position.z = v.z;
      pose.orientation = rotation;
      
      return pose;
    }
    
    void tfPublisher(string tag)
    {
      colorTF.setOrigin( tf::Vector3(trackedObject.x, trackedObject.y, trackedObject.z));
      tf::Quaternion q;
      tf::quaternionMsgToTF(trackedPose.orientation, q);
      if (q.w() == q.w())
      {
	colorTF.setRotation(q);
      }
      else
      {
	q.setW(1);
	cout << "NANNNNN" << endl;
      }
      
      br.sendTransform(tf::StampedTransform(colorTF, ros::Time::now(), "camera_rgb_optical_frame", tag));
    }

    void kinectDepthCallback(const sensor_msgs::PointCloud2::ConstPtr msg)
    {
      depth = *msg;

      //std::cout<<msg->height<<" ";
      pcl::PCLPointCloud2 pcl_pc2;
      //pcl_conversions::toPCL((sensor_msgs::PointCloud2&)msg, pcl_pc2);
      pcl_conversions::toPCL(*msg, pcl_pc2);
       //Actually convert the PointCloud2 message into a type we can reason about
      pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
    }
    
    void setNodeHandle(ros::NodeHandle* nh)
    {
      n = nh;
      pcl_cloud.width = 640;
      pcl_cloud.height = 480;
      pcl_cloud.resize(640 * 480);
    }
    
    void toggle_cb(const std_msgs::BoolConstPtr& msg)
    {
      color_switch = msg->data;
      ROS_INFO("Color Switched to %d", color_switch);
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_tracker");
  ros::NodeHandle n;
  
  colorTracker tracker;
  
  tracker.setNodeHandle(&n);
  
  tracker.color_switch = true;
  
  tracker.blobPub = n.advertise<geometry_msgs::Vector3>("/blobPt", 15);
  //tracker.blob2Pub = n.advertise<geometry_msgs::Vector3>("/bluePt", 15);
  tracker.blob3Pub = n.advertise<geometry_msgs::Vector3>("/greenPt", 15);
  
  ros::Subscriber toggleSub = n.subscribe("/color_switch", 1, &colorTracker::toggle_cb, &tracker);
  
  ros::Subscriber kinectSub = n.subscribe("/minnow/camera/depth/points", 1, &colorTracker::kinectDepthCallback, &tracker);
  ros::Subscriber blobSub = n.subscribe("/blobs", 3, &colorTracker::blobCallback, &tracker);

  //ros::Subscriber blob2Sub = n.subscribe("/blueBlobs", 1000, &colorTracker::blob2Callback, &tracker);
  
  ros::Subscriber blob3Sub = n.subscribe("/greenBlobs", 1000, &colorTracker::blob3Callback, &tracker);
  
  ros::spin();
}
