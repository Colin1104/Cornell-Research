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
#include <octomap_msgs/Octomap.h>
#include <tf/transform_broadcaster.h>
#include <octomap/octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <octomap_ros/conversions.h>




// %EndTag(INCLUDES)%
using namespace std;
using namespace octomap;

int MAX_RANGE = -1;
OcTree tree(0.01);
ros::NodeHandle n;



void pointCloudConversion(const sensor_msgs::PointCloud2& cloud) {
	Pointcloud octomapCloud;
	pointCloud2ToOctomap(cloud, octomapCloud);
	tree.insertPointCloud(octomapCloud, point3d(0,0,0), MAX_RANGE);
	tree.writeBinary("whatevs.bt");
	// ros::Publisher pub = n.advertise<OctomapBinary>("/octBinary", 1000);

	// pub.publish()
	cout << "done";
	while (true) {
		
	}

}


// %Tag(INIT)%
int main( int argc, char** argv )
{
  	ros::init(argc, argv, "octomap_kinect");
  	
  
  	ros::Subscriber sub = n.subscribe("camera/depth/points", 1000, pointCloudConversion);



  	// get the image data by subscribing to the topic
  	// Then, building the octomap from there shouldn't be too bad

	ros::spin();
	return 0;
// %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
