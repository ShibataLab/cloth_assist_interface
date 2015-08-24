#!/usr/bin/env python

# teach.py: program to enable puppet mode using Puppeteer class and record clothing trajectories
# Requirements: these pograms assume that the hook interface is connected to baxter
# Author: Nishanth Koganti
# Date: 20145/08/24
# Source: baxter_examples/scripts/joint_velocity_puppet.py

# TODO:
# 1) Consider implementing joint velocity puppet mode
# 2) Implement button interface to enable and disable puppet mode
# 3) Implement button interface for fine control of arms
# 4) Implement button interface to record via points while teaching

# import external libraries
import os
import sys
import rospy
import argparse
import baxter_interface
from std_msgs.msg import (UInt16,)
from baxter_interface import (DigitalIO,Gripper,Navigator,)

# puppeteer Class
class Puppeteer(object):
    # init function of class
    def __init__(self, limb):

        # setup variables
        puppetArm = {"left": "right", "right": "left"}

        # setup control limb and puppet limb
        self._controlLimb = limb
        self._puppetLimb = puppetArm[limb]

        # intialize baxter_interface Limb objects
        self._controlArm = baxter_interface.limb.Limb(self._controlLimb)
        self._puppetArm = baxter_interface.limb.Limb(self._puppetLimb)

        # set the software limits for Joint Velocities
        self._controlArm.set_joint_position_speed(0.6)
        self._puppetArm.set_joint_position_speed(0.6)

        # obtain the Joint names for both arms
        self._controlJointNames = self._controlArm.joint_names()
        self._puppetJointNames = self._puppetArm.joint_names()

        # obtaining robot state
        print("Getting robot state")
        self._rs = baxter_interface.RobotEnable()
        self._initState = self._rs.state().enabled

        # enable the robot
        print("Enabling robot")
        self._rs.enable()

    # function to move arms to neutral position
    def setNeutral(self):
        # move arms to neutral position
        print("Moving arms to neutral position")
        self._controlArm.move_to_neutral()
        self._puppetArm.move_to_neutral()

    # function for clean shutdown of robot
    def cleanShutdown(self):
        print("\nExiting example")

        # set the software limits back to default
        self._controlArm.set_joint_position_speed(0.4)
        self._puppetArm.set_joint_position_speed(0.4)

        # disable robot if initially disabled
        if not self._initState:
            print("Disabling robot.")
            self._rs.disable()

        # return true if this is executed
        return True

    # function to move both arms to same configuration
    def setPosition(self):

        # moving both arms to same configuration
        print("Moving both arms to same configuration")

        # set cmd dict
        cmd = {}

        # obtain control limb joint angles
        for idx, name in enumerate(self._puppetJointNames):
            j = self._controlArm.joint_angle(self._controlJointNames[idx])

            # make the joint angles same for the following joints
            if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                j = -j

            # setup the command
            cmd[name] = j

        # set the Joint Angles
        self._puppetArm.set_joint_positions(cmd)

        # sleep for some time
        rospy.sleep(2.0)

    # main function to implement puppet function
    def teachTraj(self):

        # bring arms to neutral position
        self.setPosition()

        # start Gripper Cuff Control
        arms = ('left','right')

        # set control rate to 100 Hz
        rate = rospy.Rate(100)

        # start puppeting
        print ("Cloth Assist Teaching: Grab %s cuff and move arm.") % (self._controlLimb,)
        print ("Press Ctcl-C to stop")

        # main loop
        while not rospy.is_shutdown():

            # create empty dictionary
            cmd = {}

            # obtain control arm joint angles for each joint
            for idx, name in enumerate(self._puppetJointNames):
                j = self._controlArm.joint_angle(self._controlJointNames[idx])

                # make the joint angles negative for the following joints
                if name[-2:] in ('s0', 'e0', 'w0', 'w2'):
                    j = -j

                cmd[name] = j

            # set the puppet arm joint angles
            self._puppetArm.set_joint_positions(cmd)

            # sleep function
            rate.sleep()

# main function of the program
def main():
    """Clothing Assistance Teach: Mirrors the joint angles measured on one arm to another."""

    # initialize argument parser
    argFmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=argFmt,description=main.__doc__)

    # add arguments to parser
    required = parser.add_argument_group('required arguments')
    required.add_argument("-l", "--limb",required=True,choices=['left', 'right'],help="Specify control limb")

    # parse the arguments
    args = parser.parse_args(rospy.myargv()[1:])

    # initialize ROS node
    print("Initializing node")
    rospy.init_node("ClothAssistTeach")

    # start puppet object
    puppeteer = Puppeteer(args.limb)

    # setup on shutdown function
    rospy.on_shutdown(puppeteer.cleanShutdown)

    # start puppettering
    puppeteer.teachTraj()

    # done with the experiment
    print("Done.")

    return 0

if __name__ == '__main__':
    sys.exit(main())
