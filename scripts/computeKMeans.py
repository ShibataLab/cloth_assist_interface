#!/usr/bin/env python

# computeKMeans.py: python code to compute kmeans clusters from
# tshirt point cloud data
# Requirements: baxter SDK installed and required libraries installed
# Author: Nishanth Koganti
# Date: 2015/10/18
# Source: sklearn tutorial on kmeans clustering

# import basic linear algebra and plotting libraries
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# import ml libraries
from sklearn.cluster import KMeans

# main program
def main():
    '''Program to compute KMeans clusters from tshirt point cloud data'''

    # initialize argument parser
    argFmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = argFmt, description = main.__doc__)

    # add arguments to parser
    parser.add_argument('-f', '--fileName', type = str, help = 'Root Filename')
    parser.add_argument('-c', '--clusterNum', type = int, help = 'Number of Clusters')

    # parsing arguments
    args = parser.parse_args()

    # create fileName variables
    clusterNum = args.clusterNum
    inputName = args.fileName + "Centered"
    outputName = args.fileName + "Clusters"

    # open file
    with open(inputName, "r") as fp:
        content = fp.readlines()

    # strip each line with the \n
    content = [x.strip('\n') for x in content]

    # parsing the entire file for constructing different point clouds
    lNum = 0
    frame = 0

    estimator = KMeans(n_clusters = clusterNum, init = 'k-means++', n_init = 1, max_iter = 100, tol = 1e-6)
    clustersData = np.empty((0,clusterNum*3), float)

    while lNum < len(content):
        dat = [float(val) for val in content[lNum].split(",")]
        lNum += 1

        time = dat[0]
        pNum = int(dat[1])
        cloud = np.empty([pNum, 3])

        for i in range(pNum):
            dat = [float(val) for val in content[lNum].split(",")]
            cloud[i,] = np.array(dat)
            lNum += 1

        if frame != 0:
            estimator.set_params(init = centers, n_init = 1)

        estimator.fit(cloud)
        centers = estimator.cluster_centers_
        clustersData = np.vstack([clustersData, centers.reshape((1,clusterNum*3))])

        frame += 1

    # save clustersData to file
    np.savetxt(outputName, clustersData, delimiter=",")

# call main function
if __name__ == '__main__':
    main()
