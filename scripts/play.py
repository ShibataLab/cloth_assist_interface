#!/usr/bin/env python

# play.py: python code to play baxter recorded trajectories
# Requirements: baxter SDK installed and connection to baxter robot
# Author: Nishanth Koganti
# Date: 2017/02/20
# Source: baxter_examples/scripts/joint_position_file_playback.py

# TODO:
# 1) improve method to read different types of data such as via points and play trajectory

# CHANGELOG:
# 1) Improve code to detect force threshold

# import external libraries
import os
import sys
import zmq
import time
import rospy
import argparse
import numpy as np
import baxter_interface

forceThresh = 20
bufferLength = 20

# function to check if argument is float
def tryFloat(x):
    try:
        return float(x)
    except ValueError:
        return None

# function to clean data from CSV file and return joint commands
def cleanLine(line, names):
    """Cleans single line of joint position data"""
    # convert the line to float or None
    line = [tryFloat(x) for x in line.rstrip().split(',')]

    # join the joint angle values with names
    combined = zip(names[1:], line[1:])

    # clean the tuples for any none values
    cleaned = [x for x in combined if x[1] is not None]

    # convert it to a dictionary of valid commands
    command = dict(cleaned)
    leftCommand = dict((key, command[key]) for key in command.keys() if key[:-2] == 'left_')
    rightCommand = dict((key, command[key]) for key in command.keys() if key[:-2] == 'right_')

    # return values
    return(command, leftCommand, rightCommand, line)

# function to read through CSV file and play data
def mapFile(filename):
    """Loops through given CSV File"""

    # initialize left, right objects from Limb class
    armLeft = baxter_interface.Limb('left')
    armRight = baxter_interface.Limb('right')
    armLeft.set_joint_position_speed(0.5)
    armRight.set_joint_position_speed(0.5)

    # initialize rate object from rospy Rate class
    rate = rospy.Rate(100)

    # open and read file
    print("[Baxter] Playing back: %s" % (filename,))
    with open(filename, 'r') as f:
        lines = f.readlines()

    # obtain names of columns
    keys = lines[0].rstrip().split(',')

    print("[Baxter] Moving to Start Position")

    # move to start position and start time variable
    cmdStart, lcmdStart, rcmdStart, rawStart = cleanLine(lines[1], keys)
    armLeft.move_to_joint_positions(lcmdStart)
    armRight.move_to_joint_positions(rcmdStart)
    startTime = rospy.get_time()

    # create buffers for left and right force
    leftBuffer = []
    rightBuffer = []

    # play trajectory
    i = 0
    for values in lines[1:]:
        i += 1
        sys.stdout.write("\r Record %d of %d " % (i, len(lines) - 1))
        sys.stdout.flush()

        # obtain the end effector efforts
        fL = armLeft.endpoint_effort()['force']
        fR = armRight.endpoint_effort()['force']
        fLeftRaw = np.linalg.norm([fL.x,fL.y,fL.z])
        fRightRaw = np.linalg.norm([fR.x,fR.y,fR.z])

        # append to buffer and compute moving average
        leftBuffer.append(fLeftRaw)
        rightBuffer.append(fRightRaw)
        if i >= bufferLength:
            leftBuffer.pop(0)
            rightBuffer.pop(0)

        forceLeft = np.asarray(leftBuffer).mean()
        forceRight = np.asarray(rightBuffer).mean()

        # check for force thresholds
        if forceLeft > forceThresh or forceRight > forceThresh:
            print "Error!! Force threshold exceed Left:%f, Right:%f" % (forceLeft, forceRight)
            break

        # parse line for commands
        cmd, lcmd, rcmd, values = cleanLine(values, keys)

        # execute these commands if the present time fits with time stamp
        # important implementation detail
        while (rospy.get_time() - startTime) < values[0]:

            # if we get the shutdown signal exit function
            if rospy.is_shutdown():
                print("[Baxter] Aborting - ROS shutdown")
                return False

            # execute left arm command
            if len(lcmd):
                armLeft.set_joint_positions(lcmd)

            # execute right arm command
            if len(rcmd):
                armRight.set_joint_positions(rcmd)

            # sleep command
            rate.sleep()

    # print command
    print

    # return True if there is clean exit
    return True

# main program
def main():
    """Joint Position File Playback"""

    # initialize argument parser
    argFmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = argFmt, description = main.__doc__)

    # add arguments to parser
    parser.add_argument('-f', '--fileName', type = str, help = 'Output Joint Angle Filename')
    parser.add_argument('-t', '--thresh', type = float, help = 'Force Threshold for fail detect', )

    # parsing arguments
    args = parser.parse_args(rospy.myargv()[1:])

    # initialize node with a unique name
    print("[Baxter] Initializing Node")
    rospy.init_node("PositionFilePlayback")

    # get robot state
    rs = baxter_interface.RobotEnable()
    initState = rs.state().enabled

    # define function for clean exit
    def cleanShutdown():
        print("[Baxter] Exiting example")
        if not initState:
            print("[Baxter] Disabling Robot")
            rs.disable()

    # setup the on_shutdown program
    rospy.on_shutdown(cleanShutdown)

    # enable Robot
    print("[Baxter] Enabling Robot")
    rs.enable()

    # set the force threshold if given
    if args.thresh:
        forceThresh = args.thresh

    # if optional argument is given then only play mode is run
    if args.fileName:
        mapFile(args.fileName)

    # if no arguments are given then it will run in sync mode
    else:
        context = zmq.Context()

        socket = context.socket(zmq.PAIR)

        socket.bind("tcp://*:5556")

        serverFlag = True
        while serverFlag:
            msg = socket.recv()
            if msg == "StopServer":
                print("[ZMQ] Received StopServer")
                serverFlag = False
                break

            elif msg == "NewTrial":
                print("[ZMQ] Recieved NewTrial")

                socket.send("Ready")
                print("[ZMQ] Sent Ready")

                msg = socket.recv()
                playbackFilename = msg
                print("[ZMQ] Received Playback Filename")

                # wait for start recording signal
                msg = socket.recv()
                if msg == "StartPlaying":
                    print("[ZMQ] Received StartPlaying")
                    playing = True
                    time.sleep(0.2)

                # start the mapFile function
                if playing:
                    mapFile(playbackFilename)

                # send the stop playing message
                socket.send("StoppedPlaying")
                print "[ZMQ] Sent StoppedPlaying"

        # finish
        socket.close()

    # clean shutdown
    cleanShutdown()
    print("[Baxter] Done")

if __name__ == '__main__':
    main()
