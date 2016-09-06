#! /usr/bin/env python
import roslib
import rospy
import math
import tf
import numpy as np
from tf import transformations

if __name__ == '__main__':
	rospy.init_node('vicon_to_rgbd_error_calculator')
	listener = tf.TransformListener()
	rate = rospy.Rate(5.0)
	while not rospy.is_shutdown():
		try:
			now = rospy.Time.now()
			listener.waitForTransform('/openni_rgb_optical_frame', '/map', now, rospy.Duration(4.0))
			print "boafgjdfkgdfkjkadgui"
			(errorTrans, errorRot) = listener.lookupTransform("/calibrated_rgbd_pos", "/xtion",rospy.Time.now())
			print errorTrans
			print errorRot
			print "Distance between the frames is = {0:f}".format(np.linalg.norm(errorTrans))
			print "Distance between the frames in angular is = {0:f}".format(np.linalg.norm(errorRot))
		except:
			print "transform not computed"
        # want to have octomap building at the same time in RViz
       	# then want to compute the distance from the /xtion position to the calibrated pose
       	# euclidean distance between frames and the the distance between the orientations
       	# yaw^2 + roll^2 + roll^2
        rate.sleep()