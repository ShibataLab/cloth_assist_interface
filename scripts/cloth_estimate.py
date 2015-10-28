#!/usr/bin/env python

# cloth_estimate.py: python code to subscribe cloud topic and estimate
# cloth state using MRD
# Requirements: pcl, ros and GPy installed in PC
# Author: Nishanth Koganti
# Date: 2015/10/27

# importing libraries
import os
import GPy
import sys
import time
import rospy
import random
import argparse
import numpy as np
import message_filters
import cPickle as pickle
from std_msgs.msg import Header
import matplotlib.pyplot as plt
from Interface.msg import TopCoord
from geometry_msgs.msg import Point
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# parameters for the cloth modeling
# nPlotPoints = 50
# latentInd = [0,12]
# clothModel.plot_latent(which_indices=latentInd)

# create visualizers for plotting
# cloudViz = GPy.plotting.matplot_dep.visualize.cloth_show(np.zeros((1,3)))
# esfViz = GPy.plotting.matplot_dep.visualize.vector_show(clothModel.Y0.Y[0,:])
# markerViz = GPy.plotting.matplot_dep.visualize.vector_show(clothModel.Y1.Y[0,:])

# load the trained cloth model
markerModel = pickle.load(open("markerModel.p","rb"))
topCoordModel = pickle.load(open("topCoordModel.p","rb"))

# creating the callback function
class ClothEstimator():

    # main function
    def __init__(self, nMarkers = None):
        # initialize node and setup subscriber
        rospy.init_node('cloth_estimator', anonymous=True)

        # create multiple subscribers
        cloudSub = message_filters.Subscriber('/cloth/points', PointCloud2)
        esfSub = message_filters.Subscriber('/cloth/descriptor', PointCloud2)

        ts = message_filters.TimeSynchronizer([cloudSub, esfSub], 10)
        ts.registerCallback(self.cloudCallback)

        # create the rospy publisher for marker positions
        self.markerPub = rospy.Publisher('/cloth/markers', PointCloud2, queue_size=10)
        self.topCoordPub = rospy.Publisher('/cloth/topCoord', TopCoord, queue_size=10)

        # marker points
        if nMarkers == None:
            self.nMarkers = 12
        else:
            self.nMarkers = nMarkers

        # start subscribing
        rospy.spin()

    def cloudCallback(self, cloudMsg, esfMsg):
        # parse both the messages
        gen = pc2.read_points(cloudMsg, skip_nans=True)
        cloud = np.array(list(gen))

        gen = pc2.read_points(esfMsg, skip_nans=True)
        yIn = np.array(list(gen))

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'kinect2_link'

        if yIn.shape[0] != 0:
            [xPredict1, infX1] = markerModel.Y0.infer_newX(yIn, optimize=False)
            markerOut = markerModel.predict(xPredict1.mean, Yindex=1)

            markerPoints = markerOut[0]
            markerPoints = np.reshape(markerPoints, (self.nMarkers,3))

            markers = pc2.create_cloud_xyz32(header, markerPoints)
            self.markerPub.publish(markers)

            [xPredict2, infX2] = topCoordModel.Y0.infer_newX(yIn, optimize=False)
            topCoordOut = topCoordModel.predict(xPredict2.mean, Yindex=1)

            topCoord = TopCoord()
            topCoord.header = header
            topCoord.collarHead = topCoordOut[0][0,1]
            topCoord.collarBody = topCoordOut[0][0,2]
            topCoord.leftSleeve = topCoordOut[0][0,5]
            topCoord.rightSleeve = topCoordOut[0][0,7]

            self.topCoordPub.publish(topCoord)
        else:
            print "Error!"

        # esfViz.modify(yIn)
        # markerViz.modify(yOut[0])
        # plt.draw()

        # plot the current cloud and predicted marker positions
        # ind = random.sample(xrange(cloud.shape[0]),nPlotPoints)
        # cloud = np.reshape(cloud[ind,:], (nPlotPoints*3,1))
        # cloudViz.modify(cloud)

def main():

    ce = ClothEstimator()

# main function
if __name__ == '__main__':
    main()
