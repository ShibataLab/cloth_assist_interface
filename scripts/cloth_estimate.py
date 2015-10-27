#!/usr/bin/env python

# cloth_estimate.py: python code to subscribe cloud topic and estimate
# cloth state using MRD
# Requirements: pcl, ros and GPy installed in PC
# Author: Nishanth Koganti
# Date: 2015/10/27

# importing libraries
import os
import sys
import time
import math
import rospy
import random
import argparse
import numpy as np
import message_filters
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# creating the callback function
def cloudCallback(cloudMsg, esfMsg):
    # parse both the messages
    gen = pc2.read_points(cloudMsg, skip_nans=True)
    cloud = np.array(list(gen))
    print cloud.shape

    gen = pc2.read_points(esfMsg, skip_nans=True)
    esf = np.array(list(gen))
    print esf.shape

# main function
def main():
    # initialize node and setup subscriber
    rospy.init_node('cloth_estimator')

    # create multiple subscribers
    cloudSub = message_filters.Subscriber('/cloth/points', PointCloud2)
    esfSub = message_filters.Subscriber('/cloth/descriptor', PointCloud2)

    ts = message_filters.TimeSynchronizer([cloudSub, esfSub], 10)
    ts.registerCallback(cloudCallback)

    # start subscribing
    rospy.spin()

# main function
if __name__ == '__main__':
    main()
