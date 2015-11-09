#!/usr/bin/env python

# kinect_calib.py: python code to quickly compute rough calibration
# matrix from the pose of AR tag to kinect camera
# Requirements: pcl, ros and GPy installed in PC
# Author: Nishanth Koganti
# Date: 2015/11/09

import tf
import yaml
import rospy
import numpy as np
from math import pi
from geometry_msgs.msg import Pose, Point, Quaternion

def lookupTransform(tf_listener, target, source):
    tf_listener.waitForTransform(target, source, rospy.Time(), rospy.Duration(4.0))
    trans,rot = tf_listener.lookupTransform(target, source, rospy.Time())
    return trans,rot

nFrames = 200
markernum = 4
rospy.init_node("kinect_calib")
tf_listener = tf.TransformListener()
rate = rospy.Rate(100)

Rot = np.empty((nFrames,4))
Trans = np.empty((nFrames,3))

# get the parameters for nFrames
for frame in xrange(nFrames):
    trans,rot = lookupTransform(tf_listener, '/kinect2_link', '/ar_marker_'+str(markernum))

    Rot[frame,:] = np.matrix(rot)
    Trans[frame,:] = np.matrix(trans)

    rate.sleep()

# obtain average over the values
rotMean = Rot.mean(axis=0)
transMean = Trans.mean(axis=0)

euler = tf.transformations.euler_from_quaternion(rotMean)
transMatrix = tf.transformations.compose_matrix(translate = trans, angles = euler)
np.savetxt('calibFile',transMatrix,delimiter=',')
