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
import csv

# for reference the transformations python file is located in the directory: /opt/ros/indigo/lib/python2.7/dist-packages/tf

# Want the fixed frame to be map
# want axes' reference frame to be openni_rgb_optical_frame
# view transforms
# initially, calibrated and xtion frames line up

if __name__ == '__main__':
    rospy.init_node('vicon_to_visual_odom_transform')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    flag = False
    while (not flag):
    	try:
            now = rospy.Time.now()
            listener.waitForTransform('/vicon','/kinectPB', now, rospy.Duration(4.0))
            (initialTrans, initialRot) = listener.lookupTransform('/vicon','/kinectPB', now)
            print "initialTrans: ",initialTrans
            print "initialRot: ", initialRot
            # look up transform between visual odometry and kinedt
            # (tDontCare, initialRot) = listener.lookupTransform('/kinectPB','/camera_link',now)
            # (trans1,rot1) = listener.lookupTransform('/odom','/camera_link', now)
            # (trans2,rot2) = listener.lookupTransform('/odom','/kinectPB', now)
            # deltaTrans = np.subtract(trans2,trans1)
            # deltaRot = np.subtract(rot2,rot1)
            print "timestamp, translationalError, rotError"
            flag = True
    	except tf.Exception:
    		print "transform not computed"
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            now = rospy.Time.now()
            listener.waitForTransform('/odom','/camera_link', now, rospy.Duration(4.0))
            (transODtoCL,rotODtoCL) = listener.lookupTransform('/odom','/camera_link', now)
            # transODtoCL = [transODtoCL[0] - trans[0], transODtoCL[1] - trans[1], transODtoCL[2] - trans[2]]
            # rotODtoCL = [rotODtoCL[0] - rot[0], rotODtoCL[1] - rot[1], rotODtoCL[2] - rot[2],rotODtoCL[3] - rot[3]]

            # transODtoCL = np.add(transODtoCL,deltaTrans)
            # rotODtoCL = np.add(rotODtoCL, deltaRot)

            # transODtoCL = [-trans[0] + transODtoCL[0], -trans[1] + transODtoCL[1], -trans[2] + transODtoCL[2]]
            # rotODtoCL = [rot[0] + rotODtoCL[0], rot[1] + rotODtoCL[1], rot[2] + rotODtoCL[2],rot[3] + rotODtoCL[3]]
            # print "waited"
            # br.sendTransform((0,0,0,0),initialRot,
            #    now,   "/camera_link","/calibrated_vo_pos")
            br.sendTransform(initialTrans, initialRot,
               now,  "/odom", "/vicon")
            # print ("transformed first")
            (errorTrans,errorRot) = listener.lookupTransform('/kinectPB','/camera_link', now)
            # print "timestamp: ", now
            # print "Distance between the frames is = {0:f}".format(np.linalg.norm(errorTrans))
            euler = transformations.euler_from_quaternion(errorRot,'rxyz')
            # print "Distance between the frames in angular is = {0:f}".format(np.linalg.norm(euler))
            print "{},{0:f},{0:f}".format(now, np.linalg.norm(errorTrans), np.linalg.norm(euler))
        except (tf.ExtrapolationException) as inst:
            pass
            # print type(inst)
            # print inst.args
            # print "transform not computed but inside main while loop"
        # want to have octomap building at the same time in RViz
       	# then want to compute the distance from the /kinectPB position to the calibrated pose
       	# euclidean distance between frames and the the distance between the orientations
       	# yaw^2 + roll^2 + roll^2
        
        rate.sleep()

        # Find transform between visual odometry and orientation of the kinect and take that transform and make the translation 