#! /usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import _pyvicon
import numpy as np
import tf
from tf import transformations
import math

# for reference the transformations python file is located in the directory: /opt/ros/indigo/lib/python2.7/dist-packages/tf

# Want the fixed frame to be map
# want axes' reference frame to be openni_rgb_optical_frame
# view transforms
# initially, calibrated and xtion frames line up

if __name__ == '__main__':
    rospy.init_node('vicon_to_rgbd_transform')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    flag = False
    while (not flag):
    	try:
            now = rospy.Time.now()
            listener.waitForTransform('/vicon','/kinectPB', now, rospy.Duration(4.0))

            # print "waited"
            (initialTrans, initialRot) = listener.lookupTransform('/vicon','/kinectPB', now)
            # (trans1,rot1) = listener.lookupTransform('/map','/openni_rgb_optical_frame', now)
            # (trans2,rot2) = listener.lookupTransform('/map','/kinectPB', now)
            # deltaTrans = np.subtract(trans2,trans1)
            # deltaRot = np.subtract(rot2,rot1)
            # (trans,rot) = listener.lookupTransform('/kinectPB', '/openni_rgb_optical_frame', now)
            print "timestamp, translationalError, rotError"
            flag = True
    	except (tf.ExtrapolationException, tf.Exception) as inst:
            # print type(inst)
            # print inst.args
            print "transform not computed"
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform('/map', '/openni_rgb_optical_frame', now, rospy.Duration(20.0))
            # rate.sleep()
            (transSLAMtoCL,rotSLAMtoCL) = listener.lookupTransform("/map", "/openni_rgb_optical_frame", now)
            #    now,   "/camera_link","/calibrated_vo_pos")
            br.sendTransform(initialTrans, initialRot,
               now,  "/vicon", "/map")
            # transSLAMtoCL = np.add(transSLAMtoCL, deltaTrans)
            print "waited"
            # br.sendTransform(transSLAMtoCL, rotSLAMtoCL,
            #    now, "calibrated_rgbd_pos", "/map")
            # print trans
            # print ("transformed first")
            (errorTrans,errorRot) = listener.lookupTransform('/kinectPB', '/openni_rgb_optical_frame', now)
            print "Distance between the frames is = {0:f}".format(np.linalg.norm(errorTrans))
            euler = transformations.euler_from_quaternion(errorRot,'rxyz')
            print "Distance between the frames in angular is = {0:f}".format(np.linalg.norm(euler))
            print ("transformed second")
        except (tf.ExtrapolationException, tf.Exception) as inst:
            print type(inst)
            print inst
            print "transform not computed but inside main while loop"
        # want to have octomap building at the same time in RViz
       	# then want to compute the distance from the /xtion position to the calibrated pose
       	# euclidean distance between frames and the the distance between the orientations
       	# yaw^2 + roll^2 + roll^2
        
        rate.sleep()