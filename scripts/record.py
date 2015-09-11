#!/usr/bin/env python

# record.py: python code to record baxter trajectories while teaching under gravity compensation
# Requirements: these programs assume that the hook gripper is connected to end-effector
# Author: Nishanth Koganti
# Date: 2015/08/24
# Source: baxter_examples/scripts/joint_recorder.py

# TODO:
# 1) improve recording method to only obtain via points for better skill transfer

# import external libraries
import os
import sys
import zmq
import rospy
import argparse
import baxter_interface
from recorder import JointRecorder

# main program
def main():
    """Joint Recorder Example"""

    # initialize argument parser
    argFmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class = argFmt, description = main.__doc__)

    # add arguments to parser
    parser.add_argument('-f', '--fileName', type = str, help = 'Output Joint Angle Filename')
    parser.add_argument('-m', '--mode', type = int, help = 'JointRecorder mode: 0=Only JA, 1=All data')

    # parse the arguments
    args = parser.parse_args(rospy.myargv()[1:])

    # initialize ROS node with unique name
    print("Initializing ROS Node")
    rospy.init_node("TrajectoryRecorder")

    # obtain robot state
    print("Obtaining Robot State")
    rs = baxter_interface.RobotEnable()
    initState = rs.state().enabled

    # define function for clean exit
    def cleanShutdown():
        print("\nExiting example")
        recording = False

        if not initState:
            print("Disabling Robot")
            rs.disable()

    # setup on_shutdown function
    rospy.on_shutdown(cleanShutdown)

    # enable robot
    print("Enabling the Robot")
    rs.enable()

    recordingRate = 100

    # if optional fileName argument is provided then record mode
    if args.fileName:

        # initialize joint recorder
        if args.mode:
            recorder = JointRecorder(args.fileName, recordingRate, args.mode)
        else:
            recorder = JointRecorder(args.fileName, recordingRate, 0)
        recordRate = rospy.Rate(recordingRate)

        # start recording
        print("Recording, Press Ctrl-C to stop")
        recording = True
        recorder.setTime()

        while recording:
            recorder.recordOnce()

            if rospy.is_shutdown():
                recording = False

            recordRate.sleep()

    # if no optional argument is provided then in sync mode
    else:
        # zmq initialization
        kinectPort = "5565"
        playerPort = "5556"

        context = zmq.Context()

        kinectSocket = context.socket(zmq.PAIR)
        playerSocket = context.socket(zmq.PAIR)

        kinectSocket.bind("tcp://*:%s" % kinectPort)
        playerSocket.connect("tcp://localhost:%s" % playerPort)

        # poller initialization
        poller = zmq.Poller()
        poller.register(playerSocket, zmq.POLLIN)

        # recording flag
        recording = False

        serverFlag = True
        while serverFlag:
            msg = kinectSocket.recv()
            if msg == "StopServer":
                print("[ZMQ] Received StopServer")
                serverFlag = False

                playerSocket.send("StopServer")
                print("[ZMQ] Sent StopServer")
                break

            elif msg == "NewTrial":
                print("[ZMQ] Received NewTrial")

                kinectSocket.send("Ready")
                print("[ZMQ] Sent Ready")

                msg = kinectSocket.recv()
                playbackFilename = msg
                print("[ZMQ] Received Playback Filename")

                msg = kinectSocket.recv()
                recordFilename = msg
                print("[ZMQ] Received Record Filename")

                playerSocket.send("NewTrial")
                print("[ZMQ] Sent NewTrial")

                msg = playerSocket.recv()
                if msg == "Ready":
                    print("[ZMQ] Recieved Ready")

                    playerSocket.send(playbackFilename)
                    print("[ZMQ] Sent Playback Filename")


                # initialize joint recorder
                recorder = JointRecorder(recordFilename, recordingRate, 1)
                recordRate = rospy.Rate(recordingRate)

                # wait for kinect start recording signal
                msg = kinectSocket.recv()
                if msg == "StartRecording":
                    print("[ZMQ] Received StartRecording")
                    recording = True

                playerSocket.send("StartPlaying")
                print("[ZMQ] Sent StartPlaying")

                # start recording trial
                print("[Baxter] Recording, Press Ctrl-C to stop")
                recorder.setTime()

                while recording:
                    recorder.recordOnce()

                    socks = dict(poller.poll(1))
                    if playerSocket in socks and socks[playerSocket] == zmq.POLLIN:
                        msg = playerSocket.recv()
                        if msg == "StoppedPlaying":
                            print("[ZMQ] Received StoppedPlaying")
                            recording = False

                    if rospy.is_shutdown():
                        recording = False

                kinectSocket.send("StoppedRecording")
                print("[ZMQ] Sent StoppedRecording")

        # finishing
        kinectSocket.close()
        playerSocket.close()

    # finishing
    print("Done")
    exit()

if __name__ == '__main__':
    main()
