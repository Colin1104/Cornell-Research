/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <tf/transform_broadcaster.h>
#include <octomap/octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <octomap_ros/conversions.h>
#include <octomap_server/OctomapServer.h>




// %EndTag(INCLUDES)%
using namespace std;
using namespace octomap;
using namespace octomap_msgs;

int MAX_RANGE = 3;
OcTree tree(0.01);
bool writeFile;
ros::Publisher pub;
Octomap msg;



void pointCloudConversion(const sensor_msgs::PointCloud2& cloud) {
	cout << "in callback method";
	Pointcloud octomapCloud;
	pointCloud2ToOctomap(cloud, octomapCloud);
	tree.insertPointCloud(octomapCloud, point3d(0,0,0), MAX_RANGE);
	if (writeFile) {
	  tree.writeBinary("whatevs.bt");
	  cout << "done";
	  while (true) { }
	}
	else{
	  cout << "Before issue\n";
	  msg.header.frame_id = "/camera_link";
	  msg.header.stamp = ros::Time::now();
	  tree.prune();
	  octomap_msgs::binaryMapToMsg<OcTree>(tree, msg);
	  cout << "binaryMapToMsg\n";
	  pub.publish(msg);

	  
	
	}


}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  	ros::init(argc, argv, "octomap_kinect");
	writeFile = false;
  	ros::NodeHandle n;
	pub = n.advertise<octomap_msgs::Octomap>("/octBinary", 15);
  	ros::Subscriber sub = n.subscribe("camera/depth/points", 1000, pointCloudConversion);
	


  	// get the image data by subscribing to the topic
  	// Then, building the octomap from there shouldn't be too bad

	ros::spin();
	return 0;
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
